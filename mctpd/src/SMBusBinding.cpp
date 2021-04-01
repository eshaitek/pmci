#include "SMBusBinding.hpp"

#include "MCTPBinding.hpp"

extern "C" {
#include <errno.h>
#include <i2c/smbus.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
}

#include <boost/algorithm/string.hpp>
#include <filesystem>
#include <fstream>
#include <phosphor-logging/log.hpp>
#include <regex>
#include <string>
#include <xyz/openbmc_project/MCTP/Binding/SMBus/server.hpp>

using smbus_server =
    sdbusplus::xyz::openbmc_project::MCTP::Binding::server::SMBus;

namespace fs = std::filesystem;

std::set<std::pair<int, uint8_t>> deviceMap;

static void throwRunTimeError(const std::string& err)
{
    phosphor::logging::log<phosphor::logging::level::ERR>(err.c_str());

    throw std::runtime_error(err);
}

void SMBusBinding::scanPort(const int scanFd)
{
    constexpr uint8_t startAddr = 0x03;
    constexpr uint8_t endAddr = 0x77;

    if (scanFd < 0)
    {
        phosphor::logging::log<phosphor::logging::level::ERR>(
            "Invalid I2C port fd");
        return;
    }

    for (uint8_t it = startAddr; it < endAddr; it++)
    {
        if (ioctl(scanFd, I2C_SLAVE, it) < 0)
        {
            // busy slave
            continue;
        }

        else if (i2c_smbus_read_byte(scanFd) < 0)
        {
            // no device
            continue;
        }

        /* If we are scanning a mux fd, we will encounter root bus
         * i2c devices, which needs to be part of root bus's devicemap.
         * Skip adding them to the muxfd related devicemap */

        if (scanFd != outFd)
        {
            bool flag = false;
            for (auto& device : deviceMap)
            {
                if ((std::get<0>(device) == outFd) &&
                    (std::get<1>(device) == it))
                {
                    flag = true;
                    break;
                }
            }
            if (flag)
            {
                continue;
            }
        }

        phosphor::logging::log<phosphor::logging::level::INFO>(
            ("Adding device " + std::to_string(it)).c_str());

        deviceMap.insert(std::make_pair(scanFd, it));
    }
}

/*
 * dstEid can't be removed because this is a callback passed to libmctp and we
 * have to match its expected prototype.
 */
int getSMBusOutputAddress(uint8_t /*dstEid*/, uint8_t* outAddr)
{
    // Mapping should rely on routing table and message binding private
    // Handling this here until libmctp implements routing infrastructure
    *outAddr = 0xE2; // Add in card addresses
    return 0;
}

std::optional<std::vector<uint8_t>>
    SMBusBinding::getBindingPrivateData(uint8_t dstEid)
{
    mctp_smbus_extra_params prvt = {};

    for (auto& device : smbusDeviceTable)
    {
        if (device.first == dstEid)
        {
            mctp_smbus_extra_params temp = device.second;
            prvt.fd = temp.fd;
            
                prvt.muxHoldTimeOut = 0;
                prvt.muxFlags = 0;
            
            prvt.slave_addr = temp.slave_addr;
            uint8_t* prvtPtr = reinterpret_cast<uint8_t*>(&prvt);
            return std::vector<uint8_t>(prvtPtr, prvtPtr + sizeof(prvt));
        }
    }
    return std::nullopt;
}

SMBusBinding::SMBusBinding(std::shared_ptr<object_server>& objServer,
                           std::string& objPath, ConfigurationVariant& conf,
                           boost::asio::io_context& ioc) :
    MctpBinding(objServer, objPath, conf, ioc),
    smbusReceiverFd(ioc)
{
    std::shared_ptr<dbus_interface> smbusInterface =
        objServer->add_interface(objPath, smbus_server::interface);

    try
    {
        this->arpMasterSupport =
            std::get<SMBusConfiguration>(conf).arpMasterSupport;
        this->bus = std::get<SMBusConfiguration>(conf).bus;
        this->bmcSlaveAddr = std::get<SMBusConfiguration>(conf).bmcSlaveAddr;
        this->ip = std::get<SMBusConfiguration>(conf).ip;
        registerProperty(smbusInterface, "ArpMasterSupport", arpMasterSupport);
        registerProperty(smbusInterface, "BusPath", bus);
        registerProperty(smbusInterface, "BmcSlaveAddress", bmcSlaveAddr);
        if (smbusInterface->initialize() == false)
        {
            throw std::system_error(
                std::make_error_code(std::errc::function_not_supported));
        }
    }

    catch (std::exception& e)
    {
        phosphor::logging::log<phosphor::logging::level::ERR>(
            "SMBus Interface init failed",
            phosphor::logging::entry("Exception:", e.what()));

        throw;
    }
}

void SMBusBinding::initializeBinding()
{
    try
    {
        initializeMctp();
        SMBusInit();
        io.post([this]() { initEndpointDiscovery(); });
    }

    catch (std::exception& e)
    {
        phosphor::logging::log<phosphor::logging::level::ERR>(
            "Failed to initialise SMBus binding");
    }
}

SMBusBinding::~SMBusBinding()
{
    if (smbusReceiverFd.native_handle() >= 0)
    {
        smbusReceiverFd.release();
    }
    if (inFd >= 0)
    {
        close(inFd);
    }
    if (outFd >= 0)
    {
        close(outFd);
    }
    mctp_smbus_free(smbus);
}

void SMBusBinding::SMBusInit()
{
    smbus = mctp_smbus_init();
    if (smbus == nullptr)
    {
        throwRunTimeError("Error in mctp sbus init");
    }

    if (mctp_smbus_register_bus(smbus, mctp, ownEid) != 0)
    {
        throwRunTimeError("Error in SMBus binding registration");
    }

    mctp_set_rx_all(mctp, &MctpBinding::rxMessage,
                    static_cast<MctpBinding*>(this));
/*
	FILE *pFile;
	char ip[20]={"127.0.0.1"};
	pFile = fopen("/home/mctp-srver-ip","r");
	if (pFile == NULL) {
		printf("fopen fail:%d[%s]/home/mctp-srver-ip \n",errno,strerror(errno));
	}else{
		fread(ip, 20, 1, pFile);
		fprintf(stderr,"MCTP Server IP:%s\n",ip);
		fclose(pFile);
	}
*/
    fprintf(stderr,"MCTP Server IP--:%s\n",ip.c_str());
    //socket的建立
    int sockfd_out = 0;
    sockfd_out = socket(AF_INET , SOCK_STREAM , 0);

    if (sockfd_out == -1){
        fprintf(stderr,"Fail to create a socket.\n");
    }

    //socket的連線

    struct sockaddr_in info_out;
    bzero(&info_out,sizeof(info_out));
    info_out.sin_family = PF_INET;

    //localhost test
    //info_out.sin_addr.s_addr = inet_addr("127.0.0.1");
    info_out.sin_addr.s_addr = inet_addr(ip.c_str());
    info_out.sin_port = htons(8700);


    int err = connect(sockfd_out, reinterpret_cast< struct sockaddr *>(&info_out),sizeof(info_out));
    if(err==-1){
       fprintf(stderr,"Connection 8700 error\n");
    }else
        outFd = sockfd_out;


    mctp_smbus_set_out_fd(smbus, outFd);

    //socket的建立
    int sockfd_in = 0;
    sockfd_in = socket(AF_INET , SOCK_STREAM , 0);

    if (sockfd_in == -1){
        fprintf(stderr,"Fail to create a socket.\n");
    }

    //socket的連線

    struct sockaddr_in info_in;
    bzero(&info_in,sizeof(info_in));
    info_in.sin_family = PF_INET;

    //localhost test
    info_in.sin_addr.s_addr = inet_addr(ip.c_str());
    info_in.sin_port = htons(8701);


    err = connect(sockfd_in, reinterpret_cast< struct sockaddr *>(&info_in),sizeof(info_in));
    if(err==-1)
       fprintf(stderr,"Connection to 8701 error\n");
    else
        inFd = sockfd_in;

    mctp_smbus_set_in_fd(smbus, inFd);


    smbusReceiverFd.assign(inFd);
    //readResponse();
}

void SMBusBinding::readResponse()
{
#if 0

static constexpr size_t pollTime = 0; // in seconds

    waitTimer.expires_from_now(boost::posix_time::seconds(pollTime));

    waitTimer.async_wait([this](const boost::system::error_code& ec) {

        fprintf(stderr,"SMBusBinding::readResponse\n");

        if (ec == boost::asio::error::operation_aborted)
        {
            fprintf(stderr,"we're being cancelled\n");
            return; // we're being cancelled
        }
        // read timer error
        else if (ec)
        {
            fprintf(stderr,"timer error in sensor_read_loop\n");
            return;
        }

        mctp_smbus_read(smbus);

        readResponse();
    });

#else
    smbusReceiverFd.async_wait(
        boost::asio::posix::descriptor_base::wait_error, [this](auto& ec) {
            if (ec)
            {
                phosphor::logging::log<phosphor::logging::level::ERR>(
                    "Error: mctp_smbus_read()");
                readResponse();
            }
            // through libmctp this will invoke rxMessage and message assembly
            mctp_smbus_read(smbus);

            readResponse();
        });
#endif
}

void SMBusBinding::scanAllPorts()
{
    phosphor::logging::log<phosphor::logging::level::INFO>(
        "Scanning root port");
    // Scan rootbus
    uint8_t it = 0x77;

    phosphor::logging::log<phosphor::logging::level::INFO>(
            ("Adding device " + std::to_string(it)).c_str());

    deviceMap.insert(std::make_pair(outFd, it));

}

bool SMBusBinding::isMuxFd(const int fd)
{
    for (auto& muxFd : muxFds)
    {
        if (fd == std::get<1>(muxFd))
        {
            return true;
        }
    }
    return false;
}

void SMBusBinding::initEndpointDiscovery()
{
    phosphor::logging::log<phosphor::logging::level::INFO>(
        "InitEndpointDiscovery");

    uint8_t it = 0x71;

    phosphor::logging::log<phosphor::logging::level::INFO>(
            ("1--Adding device " + std::to_string(it)).c_str());

    deviceMap.insert(std::make_pair(outFd, it));

    /* Since i2c muxes restrict that only one command needs to be
     * in flight, we cannot register multiple endpoints in parallel.
     * Thus, in a single yield_context, all the discovered devices
     * are attempted with registration sequentially */

    boost::asio::spawn(io, [this](boost::asio::yield_context yield) {
        for (auto& device : deviceMap)
        {
            phosphor::logging::log<phosphor::logging::level::INFO>(
                ("Checking if device " + std::to_string(std::get<1>(device)) +
                 " is MCTP Capable")
                    .c_str());

            struct mctp_smbus_extra_params smbusBindingPvt;
            smbusBindingPvt.fd = std::get<0>(device);

            {
                smbusBindingPvt.muxHoldTimeOut = 0;
                smbusBindingPvt.muxFlags = 0;
            }
            /* Set 8 bit i2c slave address */
            smbusBindingPvt.slave_addr =
                static_cast<uint8_t>((std::get<1>(device) << 1));

            auto const ptr = reinterpret_cast<uint8_t*>(&smbusBindingPvt);
            std::vector<uint8_t> bindingPvtVect(ptr,
                                                ptr + sizeof(smbusBindingPvt));
            mctp_eid_t eid = 0xFF;
            if( smbusBindingPvt.slave_addr == 0xE2)//This is a PLDM I2C Slave device on NCT6681 and NCT6692
                eid = 0x08;
            else
                fprintf(stderr,"slave_addr:%X in deviceMap\n", smbusBindingPvt.slave_addr);
            auto rc = registerEndpoint(yield, bindingPvtVect, eid);
            if (rc)
            {
                fprintf(stderr,"push EID:%d slave_addr:%X in smbusDeviceTable\n", eid, smbusBindingPvt.slave_addr);
                smbusDeviceTable.push_back(
                    std::make_pair(eid, smbusBindingPvt));
            }
        }
    });
}

// TODO: This method is a placeholder and has not been tested
bool SMBusBinding::handleGetEndpointId(mctp_eid_t destEid, void* bindingPrivate,
                                       std::vector<uint8_t>& request,
                                       std::vector<uint8_t>& response)
{
    if (!MctpBinding::handleGetEndpointId(destEid, bindingPrivate, request,
                                          response))
    {
        return false;
    }

    auto const ptr = reinterpret_cast<uint8_t*>(bindingPrivate);
    if (auto bindingPvtVect = getBindingPrivateData(destEid))
    {
        std::copy(bindingPvtVect->begin(), bindingPvtVect->end(), ptr);
        return true;
    }
    return false;
}
