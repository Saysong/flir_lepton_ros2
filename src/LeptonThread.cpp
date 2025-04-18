#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "flir_lepton/LeptonThread.h"
#include "std_srvs/srv/empty.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "Lepton3.hpp"
#include <cv_bridge/cv_bridge.hpp>

using namespace std;

// ----> Global variables
Lepton3* lepton3=nullptr;
// static bool close_var = false;
static bool rgb_mode = true;
// <---- Global variables

// ----> Global functions
void close_handler(int s);
void keyboard_handler(int key);

void set_rgb_mode(bool enable);
// <---- Global functions

// int param_value;
// node->declare_parameter("typeLepton", default_value);
// node->get_parameter("typeLepton", param_value);

LeptonThread::LeptonThread()
{
	//
	loglevel = 0;

	//
	typeColormap = 3; // 1:colormap_rainbow  /  2:colormap_grayscale  /  3:colormap_ironblack(default)
	// selectedColormap = colormap_ironblack;
	// selectedColormapSize = get_size_colormap_ironblack();

	//
	typeLepton = 3; // 2:Lepton 2.x  / 3:Lepton 3.x
	myImageWidth = 160;
	myImageHeight = 120;

	//
	spiSpeed = 20 * 1000 * 1000; // SPI bus speed 20MHz

	// min/max value for scaling
	autoRangeMin = true;
	autoRangeMax = true;
	rangeMin = 30000;
	rangeMax = 32000;

	imgCount = 0;

}

LeptonThread::LeptonThread(rclcpp::Node::SharedPtr n)
{
    node = n;
    loglevel = 0;
    typeColormap = 3;
    // selectedColormap = colormap_ironblack;
    // selectedColormapSize = get_size_colormap_ironblack();

    typeLepton = 3;
    myImageWidth = 160;
    myImageHeight = 120;

    spiSpeed = 20 * 1000 * 1000;
    autoRangeMin = true;
    autoRangeMax = true;
    rangeMin = 30000;
    rangeMax = 32000;
    imgCount = 0;
}


LeptonThread::~LeptonThread() {

}

void LeptonThread::setLogLevel(uint16_t newLoglevel)
{
	loglevel = newLoglevel;
}

void LeptonThread::useColormap(int newTypeColormap)
{
	switch (newTypeColormap)
	{
		case 1:
			typeColormap = 1;
			// selectedColormap = colormap_rainbow;
			// selectedColormapSize = get_size_colormap_rainbow();
			break;
		case 2:
			typeColormap = 2;
			// selectedColormap = colormap_grayscale;
			// selectedColormapSize = get_size_colormap_grayscale();
			break;
		default:
			typeColormap = 3;
			// selectedColormap = colormap_ironblack;
			// selectedColormapSize = get_size_colormap_ironblack();
			break;
	}
}

void LeptonThread::useLepton(int newTypeLepton)
{
	switch (newTypeLepton)
	{
		case 3:
			typeLepton = 3;
			myImageWidth = 160;
			myImageHeight = 120;
			break;
		default:
			typeLepton = 2;
			myImageWidth = 80;
			myImageHeight = 60;
	}
}

void LeptonThread::useSpiSpeedMhz(unsigned int newSpiSpeed)
{
	spiSpeed = newSpiSpeed * 1000 * 1000;
}

void LeptonThread::setAutomaticScalingRange(int enabled) {
  if(enabled == 1) {
    setAutomaticScalingRange();
  } else {
    autoRangeMin = false;
  	autoRangeMax = false;
  }
}

void LeptonThread::setAutomaticScalingRange()
{
	autoRangeMin = true;
	autoRangeMax = true;
}

void LeptonThread::useRangeMinValue(uint16_t newMinValue)
{
	autoRangeMin = false;
	rangeMin = newMinValue;
}

void LeptonThread::useRangeMaxValue(uint16_t newMaxValue)
{
	autoRangeMax = false;
	rangeMax = newMaxValue;
}

void LeptonThread::setPublisher(rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub) 
{
    publisherImage = pub;
}

void LeptonThread::setNode(rclcpp::Node::SharedPtr n) {
    node = n;
}

void LeptonThread::run()
{
    RCLCPP_INFO(node->get_logger(), "Lepton reading...");
	//create the initial image
	myImage = cv::Mat(myImageHeight, myImageWidth, CV_8UC3); // create image with 3 channels (RGB)

	const int *colormap = selectedColormap;
	const int colormapSize = selectedColormapSize;
	uint16_t minValue = rangeMin;
	uint16_t maxValue = rangeMax;
	float diff = maxValue - minValue;
	float scale = 255/diff;
	uint16_t n_wrong_segment = 0;
	uint16_t n_zero_value_drop_frame = 0;
    //open spi port
	// SpiOpenPort(0, spiSpeed);
	// struct sigaction sigIntHandler;
    // sigIntHandler.sa_handler = close_handler;
    // sigemptyset(&sigIntHandler.sa_mask);
    // sigIntHandler.sa_flags = 0;
    // sigaction(SIGINT, &sigIntHandler, NULL);
    // <---- Set Ctrl+C handler

    Lepton3::DebugLvl deb_lvl = Lepton3::DBG_NONE;

    lepton3 = new Lepton3( "/dev/spidev0.0", "/dev/i2c-1", deb_lvl ); // use SPI1 and I2C-1 ports
    lepton3->start();

    // Set initial data mode
    set_rgb_mode(rgb_mode);

    uint64_t frameIdx=0;
    uint16_t min;
    uint16_t max;
    uint8_t w,h;

    StopWatch stpWtc;

    stpWtc.tic();

	rclcpp::Rate rate(25);
	while(rclcpp::ok()) {

		//read data packets from lepton over SPI
		int resets = 0;
		int segmentNumber = -1;
		// for(int j=0;j<PACKETS_PER_FRAME;j++) {
		// 	//if it's a drop packet, reset j to 0, set to -1 so he'll be at 0 again loop
		// 	read(spi_cs0_fd, result+sizeof(uint8_t)*PACKET_SIZE*j, sizeof(uint8_t)*PACKET_SIZE);
		// 	int packetNumber = result[j*PACKET_SIZE+1];
		// 	if(packetNumber != j) {
		// 		j = -1;
		// 		resets += 1;
		// 		usleep(1000);
		// 		//Note: we've selected 750 resets as an arbitrary limit, since there should never be 750 "null" packets between two valid transmissions at the current poll rate
		// 		//By polling faster, developers may easily exceed this count, and the down period between frames may then be flagged as a loss of sync
		// 		if(resets == 750) {
		// 			SpiClosePort(0);
		// 			lepton_reboot();
		// 			n_wrong_segment = 0;
		// 			n_zero_value_drop_frame = 0;
		// 			usleep(750000);
		// 			SpiOpenPort(0, spiSpeed);
		// 		}
		// 		continue;
		// 	}
		// 	if ((typeLepton == 3) && (packetNumber == 20)) {
		// 		segmentNumber = (result[j*PACKET_SIZE] >> 4) & 0x0f;	// using 4-segment

		// 		// check the segment number
		// 		// if -1 or 0 is printed, it's not segment structure
		// 		// if 1, 2, 3, or 4 is printed, it's segment structure
		// 		RCLCPP_INFO(node->get_logger(), "Segment number: %d", segmentNumber);

		// 		if ((segmentNumber < 1) || (4 < segmentNumber)) {
		// 			log_message(10, "[ERROR] Wrong segment number " + std::to_string(segmentNumber));
		// 			break;
		// 		}
		// 	}
		// }
		// if(resets >= 30) {
		// 	log_message(3, "done reading, resets: " + std::to_string(resets));
		// }


		// //
		// int iSegmentStart = 1;
		// int iSegmentStop;

		// if (typeLepton == 3) {
		// 	if ((segmentNumber < 1) || (4 < segmentNumber)) {
		// 		n_wrong_segment++;
		// 		if ((n_wrong_segment % 12) == 0) {
		// 			log_message(5, "[WARNING] Got wrong segment number continuously " + std::to_string(n_wrong_segment) + " times");
		// 		}
		// 		continue;
		// 	}
		// 	if (n_wrong_segment != 0) {
		// 		log_message(8, "[WARNING] Got wrong segment number continuously " + std::to_string(n_wrong_segment) + " times [RECOVERED] : " + std::to_string(segmentNumber));
		// 		n_wrong_segment = 0;
		// 	}

		// 	//
		// 	memcpy(shelf[segmentNumber - 1], result, sizeof(uint8_t) * PACKET_SIZE*PACKETS_PER_FRAME);
		// 	if (segmentNumber != 4) {
		// 		continue;
		// 	}
		// 	iSegmentStop = 4;
		// }
		// else {
		// 	memcpy(shelf[0], result, sizeof(uint8_t) * PACKET_SIZE*PACKETS_PER_FRAME);
		// 	iSegmentStop = 1;
		// }

		// if ((autoRangeMin == true) || (autoRangeMax == true)) {
		// 	if (autoRangeMin == true) {
		// 		maxValue = 65535;
		// 	}
		// 	if (autoRangeMax == true) {
		// 		maxValue = 0;
		// 	}
		// 	for(int iSegment = iSegmentStart; iSegment <= iSegmentStop; iSegment++) {
		// 		for(int i=0;i<FRAME_SIZE_UINT16;i++) {
		// 			//skip the first 2 uint16_t's of every packet, they're 4 header bytes
		// 			if(i % PACKET_SIZE_UINT16 < 2) {
		// 				continue;
		// 			}

		// 			//flip the MSB and LSB at the last second
		// 			uint16_t value = (shelf[iSegment - 1][i*2] << 8) + shelf[iSegment - 1][i*2+1];
		// 			if (value == 0) {
		// 				// Why this value is 0?
		// 				continue;
		// 			}
		// 			if ((autoRangeMax == true) && (value > maxValue)) {
		// 				maxValue = value;
		// 			}
		// 			if ((autoRangeMin == true) && (value < minValue)) {
		// 				minValue = value;
		// 			}
		// 		}
		// 	}
		// 	diff = maxValue - minValue;
		// 	scale = 255/diff;
		// }

		// int row, column;
		// uint16_t value;
		// uint16_t valueFrameBuffer;
		// cv::Vec3b color;
		// for(int iSegment = iSegmentStart; iSegment <= iSegmentStop; iSegment++) {
		// 	int ofsRow = 30 * (iSegment - 1);
		// 	for(int i=0;i<FRAME_SIZE_UINT16;i++) {
		// 		//skip the first 2 uint16_t's of every packet, they're 4 header bytes
		// 		if(i % PACKET_SIZE_UINT16 < 2) {
		// 			continue;
		// 		}

		// 		//flip the MSB and LSB at the last second
		// 		valueFrameBuffer = (shelf[iSegment - 1][i*2] << 8) + shelf[iSegment - 1][i*2+1];
		// 		if (valueFrameBuffer == 0) {
		// 			// Why this value is 0?
		// 			n_zero_value_drop_frame++;
		// 			if ((n_zero_value_drop_frame % 12) == 0) {
		// 				log_message(5, "[WARNING] Found zero-value. Drop the frame continuously " + std::to_string(n_zero_value_drop_frame) + " times");
		// 			}
		// 			break;
		// 		}

		// 		value = (valueFrameBuffer - minValue) * scale;
		// 		int ofs_r = 3 * value + 0; if (colormapSize <= ofs_r) ofs_r = colormapSize - 1;
		// 		int ofs_g = 3 * value + 1; if (colormapSize <= ofs_g) ofs_g = colormapSize - 1;
		// 		int ofs_b = 3 * value + 2; if (colormapSize <= ofs_b) ofs_b = colormapSize - 1;
		// 		color = cv::Vec3b(colormap[ofs_r], colormap[ofs_g], colormap[ofs_b]);	// apply color map to the raw value


		// 		// check the row/column here!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
		// 		// if (typeLepton == 3) {
		// 		// 	column = (i % PACKET_SIZE_UINT16) - 2 + (myImageWidth / 2) * ((i % (PACKET_SIZE_UINT16 * 2)) / PACKET_SIZE_UINT16);
		// 		// 	row = i / PACKET_SIZE_UINT16 / 2 + ofsRow;
		// 		// }
		// 		// else {
		// 		// 	column = (i % PACKET_SIZE_UINT16) - 2;
		// 		// 	row = i / PACKET_SIZE_UINT16;
		// 		// }
		// 		// This calculation is for Lepton 3.5
		// 		int pixel_index = i % PACKET_SIZE_UINT16; // extract pixel index within the packet, skipping 2 header bytes
		// 		int packet_index = i / PACKET_SIZE_UINT16; // determine which packet in the sequence this pixel came from

		// 		// column position includes horizontal shift depending on even/odd row pairs.
		// 		// `(myImageWidth / 2)` handles horizontal interleaving.
		// 		column = (pixel_index - 2) + (myImageWidth / 2) * ((i % (PACKET_SIZE_UINT16 * 2)) / PACKET_SIZE_UINT16);
		// 		row = packet_index / 2 + ofsRow; // row is adjusted by the segment offset (each segment adds 30 rows).

		// 		// ensure with the image size
		// 		if (column < 0 || column >= myImageWidth || row < 0 || row >= myImageHeight) continue;

		// 		myImage.at<cv::Vec3b>(row, column) = color; // write the pixel value into the image. We need this line.
		// 	}
		// }

		// if (n_zero_value_drop_frame != 0) {
		// 	log_message(8, "[WARNING] Found zero-value. Drop the frame continuously " + std::to_string(n_zero_value_drop_frame) + " times [RECOVERED]");
		// 	n_zero_value_drop_frame = 0;
		// }

		//lets emit the signal for update
		const uint8_t* dataRGB = lepton3->getLastFrameRGB( w, h );
		if (!dataRGB) {
			log_message(5, "[ERROR] No data from Lepton3");
		}
		else {
			memcpy(myImage.data, dataRGB, 3*w*h*sizeof(uint8_t) ); // copy the RGB data to the image
	
			publishImage();
		}
    //rclcpp::spinOnce();
    rate.sleep();
	}
	delete lepton3;
  	//finally, close SPI port just bcuz
	// SpiClosePort(0);
}

void LeptonThread::publishImage()
{
  rclcpp::Time timestamp = node->get_clock()->now();
    if (publisherImage) {
        cv_bridge::CvImage img_bridge;
        sensor_msgs::msg::Image img_msg;
        std_msgs::msg::Header header;
        header.stamp = timestamp;
        img_bridge = cv_bridge::CvImage(header, "rgb8", myImage);	// using a rgb8 encoding; publish okay.
        img_bridge.toImageMsg(img_msg);
        publisherImage->publish(img_msg);
        RCLCPP_INFO(node->get_logger(), "published thermal image");
    }
}

void LeptonThread::log_message(uint16_t level, std::string msg)
{
	if (level <= loglevel) {
		RCLCPP_ERROR(rclcpp::get_logger("flir_lepton"), "%s", msg.c_str());
	}
}

// fixed for ROS2
bool LeptonThread::performFFC(
    const std::shared_ptr<std_srvs::srv::Empty::Request> request,
    std::shared_ptr<std_srvs::srv::Empty::Response> response)
{
    (void)request;
    (void)response;

    // lepton_perform_ffc();
    RCLCPP_INFO(rclcpp::get_logger("flir_lepton"), "Lepton Perform FFC.");
    return true;
}



void close_handler(int s)
{
    if(s==2)
    {
        cout << std::endl << "Ctrl+C pressed..." << std::endl;
        // close_var = true;
    }
}

void keyboard_handler(int key)
{
    switch(key)
    {

    case 'c': // Set RGB mode
        set_rgb_mode(true);
        break;

    case 'r': // Set radiometry mode
        set_rgb_mode(false);
        break;

    case 'h':
        if( lepton3->setGainMode( LEP_SYS_GAIN_MODE_HIGH ) == LEP_OK )
        {

            LEP_SYS_GAIN_MODE_E gainMode;
            if( lepton3->getGainMode( gainMode ) == LEP_OK )
            {
                string str = (gainMode==LEP_SYS_GAIN_MODE_HIGH)?string("High"):((gainMode==LEP_SYS_GAIN_MODE_LOW)?string("Low"):string("Auto"));
                cout << " * Gain mode: " << str << std::endl;
            }
        }
        break;

    case 'l':
        if( lepton3->setGainMode( LEP_SYS_GAIN_MODE_LOW ) == LEP_OK )
        {

            LEP_SYS_GAIN_MODE_E gainMode;
            if( lepton3->getGainMode( gainMode ) == LEP_OK )
            {
                string str = (gainMode==LEP_SYS_GAIN_MODE_HIGH)?string("High"):((gainMode==LEP_SYS_GAIN_MODE_LOW)?string("Low"):string("Auto"));
                cout << " * Gain mode: " << str << std::endl;
            }
        }
        break;

    case 'a':
        if( lepton3->setGainMode( LEP_SYS_GAIN_MODE_AUTO ) == LEP_OK )
        {

            LEP_SYS_GAIN_MODE_E gainMode;
            if( lepton3->getGainMode( gainMode ) == LEP_OK )
            {
                string str = (gainMode==LEP_SYS_GAIN_MODE_HIGH)?string("High"):((gainMode==LEP_SYS_GAIN_MODE_LOW)?string("Low"):string("Auto"));
                cout << " * Gain mode: " << str << std::endl;
            }
        }
        break;

    case 'f':
        if( lepton3->doFFC() == LEP_OK )
        {
            cout << " * FFC completed" << std::endl;
        }
        break;

    case 'F':
        if( lepton3->doRadFFC() == LEP_OK )
        {
            cout << " * Radiometry FFC completed" << std::endl;
        }
        break;

    default:
        break;
    }
}

void set_rgb_mode(bool enable)
{
    rgb_mode = enable;

    if( lepton3->enableRadiometry( !rgb_mode ) < 0)
    {
        cerr << "Failed to set radiometry status" << std::endl;
    }
    else
    {
        if(!rgb_mode)
        {
            cout << " * Radiometry enabled " << std::endl;
        }
        else
        {
            cout << " * Radiometry disabled " << std::endl;
        }
    }

    // NOTE: if radiometry is enabled is unuseful to keep AGC enabled
    //       (see "FLIR LEPTON 3® Long Wave Infrared (LWIR) Datasheet" for more info)

    if( lepton3->enableAgc( rgb_mode ) < 0)
    {
        cerr << "Failed to set radiometry status" << std::endl;
    }
    else
    {
        if(!rgb_mode)
        {
            cout << " * AGC disabled " << std::endl;
        }
        else
        {
            cout << " * AGC enabled " << std::endl;
        }
    }

    if( lepton3->enableRgbOutput( rgb_mode ) < 0 )
    {
        cerr << "Failed to enable RGB output" << std::endl;
    }
    else
    {
        if(rgb_mode)
        {
            cout << " * RGB enabled " << std::endl;
        }
        else
        {
            cout << " * RGB disabled " << std::endl;
        }
    }
}
