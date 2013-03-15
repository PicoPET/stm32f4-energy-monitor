#include <stdio.h>
#include <libusb-1.0/libusb.h>
#include "libusbinterface.h"
#include <string>
#include <boost/bind.hpp>
#include <time.h>
#include <signal.h>
#include "helper.h"

using namespace boost;
// using namespace boost::bind;
using namespace std;

LibusbInterface::LibusbInterface(boost::mutex *m, queue<shared_array<unsigned char> > *d,
    unsigned idVendor, unsigned idProduct, string serialId)
{
    mQueue = m;
    dQueue = d;
    this->idVendor = idVendor;
    this->idProduct = idProduct;
    this->serialId = serialId;
    status = 0;
    total_len = 0;
    running = false;
}

LibusbInterface::~LibusbInterface()
{
    printf("Destructr\n");
}

std::vector<std::pair<std::string, std::string> > LibusbInterface::listDevices(unsigned idVendor, unsigned idProduct)
{
    libusb_device **devs, *dev;
    int i = 0, r;
    ssize_t cnt;
    vector<pair<string,string> > devlist;

    cnt = libusb_get_device_list(NULL, &devs);
    if (cnt < 0)
    {
        printf("Could not get device list: %d\n", (int)cnt);
        exit(1);
    }

    while ((dev = devs[i++]) != NULL)
    {
        struct libusb_device_descriptor desc;
        libusb_device_handle *devh;

        int r = libusb_get_device_descriptor(dev, &desc);
        if (r < 0)
        {
            printf("Failed to get device descriptor: %d\n", r);
            continue;
        }

        if(desc.idVendor == idVendor && desc.idProduct == idProduct)
        {
            unsigned char sId[256];

            r = libusb_open(dev, &devh);
            if(r < 0)
            {
                printf("Failed to open device: %d\n", r);
                continue;
            }

            r = libusb_get_string_descriptor_ascii(devh, desc.iSerialNumber, sId, 256);
            if(r < 0)
            {
                printf("Failed to get string descriptor: %d\n", r);
                continue;
            }

            devlist.push_back(make_pair(std::string((char*)sId), ""));

            libusb_close(devh);
        }
    }

    return devlist;
}


void LibusbInterface::operator()()
{
    int r;
    int t1, t2;

    open_device();

    energy_transfer = libusb_alloc_transfer(0);
    if(!energy_transfer)
    {
        printf("Couldn't allocate a transfer\n");
        return;
    }

    libusb_fill_bulk_transfer(energy_transfer, devh, (1 | LIBUSB_ENDPOINT_IN), data_buf, sizeof(data_buf), &LibusbInterface::transfer_callback, this, 100);

    // if(status == 0)
    //     sendCommand(START);

    t1 = time(0);
    while(status == 0)
    {
        timeval tv;
        tv.tv_sec = 0;
        tv.tv_usec = 100000;
        r = libusb_handle_events_timeout(NULL, &tv);

        if(r < 0)
        {
            printf("Error?\n");
            status = -1;
        }

        t2 =  time(0);

        if(t2 - t1 >= 2)
        {
            mt_start_output();
            printf("%f\n", (float)total_len/(t2-t1));
            mt_end_output();
            t1=t2;
            total_len = 0;
        }

        if(!cQueue.empty())
        {
            boost::mutex::scoped_lock(cQueueMutex);
            if(sendMonitorCommand(cQueue.front()))
                cQueue.pop();
        }
    }

    if(devh)
    {
        CommandData cd = {STOP, ""};
        while(!sendMonitorCommand(cd));
    }

    close_device();
}

bool LibusbInterface::open_device()
{
    libusb_device **devs, *dev;
    int i = 0, r;
    ssize_t cnt;

    r = libusb_init(NULL);
    if (r < 0)
    {
        printf("Could not initialise libusb: %d\n", r);
        status = -1;
        return false;
    }

    cnt = libusb_get_device_list(NULL, &devs);
    if (cnt < 0)
    {
        printf("Could not get device list: %d\n", (int)cnt);
        status = -1;
        return false;
    }

    while ((dev = devs[i++]) != NULL)
    {
        struct libusb_device_descriptor desc;

        int r = libusb_get_device_descriptor(dev, &desc);
        if (r < 0)
        {
            printf("Failed to get device descriptor: %d\n", r);
            status = -1;
            return false;
        }

        if(desc.idVendor == idVendor && desc.idProduct == idProduct)
        {
            unsigned char sId[256];

            r = libusb_open(dev, &devh);
            if(r < 0)
            {
                printf("Failed to open device: %d\n", r);
                status = -1;
                return false;
            }

            r = libusb_get_string_descriptor_ascii(devh, desc.iSerialNumber, sId, 256);
            if(r < 0)
            {
                printf("Failed to get string descriptor: %d\n", r);
                status = -1;
                return false;
            }

            if(serialId == std::string((char*)sId))
            {
                r = libusb_claim_interface(devh, 0);
                if(r < 0)
                {
                    printf("Error claiming interface %d\n", r);
                    status = -1;
                    return false;
                }

                return true;
            }
        }
    }

    printf("Could not find device...\n");
    status = -1;
    return false;
}

void LibusbInterface::send_start()
{
    unsigned char buf[] = "S";
    int len;

    libusb_bulk_transfer(devh, 0x1, buf, 1, &len, 0);
}

void LibusbInterface::send_end()
{
    unsigned char buf[] = "F";
    int len;

    libusb_bulk_transfer(devh, 0x1, buf, 1, &len, 0);
}

void LibusbInterface::close_device()
{
    if(devh)
    {
        libusb_release_interface(devh, 0);
        libusb_close(devh);
    }
}

void LibusbInterface::send_data(shared_array<unsigned char> data)
{
    boost::mutex::scoped_lock(*mQueue);

    dQueue->push(data);
}


void LIBUSB_CALL LibusbInterface::transfer_callback(struct libusb_transfer *transfer)
{
    LibusbInterface * _this = (LibusbInterface*)transfer->user_data;
    int r;

    if(transfer->status != LIBUSB_TRANSFER_COMPLETED)
    {
        if(transfer->status == LIBUSB_TRANSFER_CANCELLED)
            return;
        printf("Transfer status: %d\n", transfer->status);
        libusb_free_transfer(transfer);
        _this->energy_transfer = NULL;
        return;
    }

    // printf("cb\n");
    _this->total_len += DATA_LEN;

    shared_array<unsigned char> data(new unsigned char[DATA_LEN]);

    memcpy(data.get(), _this->data_buf, DATA_LEN);
    _this->send_data(data);

    if((r=libusb_submit_transfer(transfer)) < 0)
    {
        printf("Submit transfer error: %d\n", r);
        return;
    }
}

void LibusbInterface::endSignal()
{
    status = 1;
}

void LibusbInterface::sendCommand(CommandType cmd)
{
    boost::mutex::scoped_lock(cQueueMutex);
    LibusbInterface::CommandData cd = {cmd, ""};
    cQueue.push(cd);
}

void LibusbInterface::setSerial(string ser)
{
    boost::mutex::scoped_lock(cQueueMutex);
    LibusbInterface::CommandData cd = {SETSERIAL, ser};
    cQueue.push(cd);
}

bool LibusbInterface::sendMonitorCommand(CommandData cmd)
{
    int r;

    if(cmd.cmd == SETSERIAL)
    {
        unsigned char data[8] = {0};
        int len = cmd.cmd_data.length();

        if(len > 8)
            len = 8;

        memcpy(data, cmd.cmd_data.c_str(), len);

        r = libusb_control_transfer(devh, 0x41, cmd.cmd, 0, 0, data, len, 3000);
    }
    else
    {
        r = libusb_control_transfer(devh, 0x41, cmd.cmd, 0, 0, NULL, 0, 100);
    }

    if(r < 0)
    {
        printf("Failed to send cmd: %d\n", r);
        return false;
    }

    if(cmd.cmd == STOP)
    {
        if(running)
            libusb_cancel_transfer(energy_transfer);
        running = false;
    }
    if(cmd.cmd == START)
    {
        if(!running)
            libusb_submit_transfer(energy_transfer);
        running = true;
    }

    return true;
}
