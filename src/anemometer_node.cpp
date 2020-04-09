#include <ros/ros.h>
#include <iomanip>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include <anemometer/ImuConverged.h>
#include <geometry_msgs/Twist.h> //Message Type out : Velocity
#include <ctime> //needed for time() in random number generator
#include <unistd.h>
#include <sensor_msgs/Imu.h>
#include <fcntl.h>  /* File Control Definitions          */
#include <termios.h>/* POSIX Terminal Control Definitions*/
#include <unistd.h> /* UNIX Standard Definitions         */
#include <errno.h>  /* ERROR Number Definitions          */
#include <sys/ioctl.h>
#include <cmath>

using namespace std;

//--------------Class------------------------------
class windData
{
  ros::Subscriber sb_imu;
  ros::Publisher pb_merged;
  ros::NodeHandle nh;

  int fd;

  // class callback function that reads IMU data from pixhawk
  void windGrab(const sensor_msgs::Imu msg);

  public: 
  
    windData();
	~windData(){close(fd);}  //properly closes anemometer serial port

};
//----------------Constructor----------------------------
windData::windData()
{

	/*For experimenting with other port settings*/
	// fd = open("/dev/ttyUSB3",O_RDWR | O_NOCTTY | O_NONBLOCK | O_SYNC);
	// fd = open("/dev/ttyUSB3",O_RDWR | O_NOCTTY | O_NONBLOCK);
	// fd = open("/dev/ttyUSB3",O_RDWR | O_NOCTTY | O_NDELAY);
	// fd = open("/dev/ttyUSB3",O_RDWR | O_NOCTTY);

	fd = open("/dev/ttyUSB0",O_RDWR | O_NOCTTY | O_NONBLOCK);

    if(fd == -1) // Check if port was ever opened 
           printf("\n  Error! in Opening ttyUSB0  ");
    else
           printf("\n  ttyUSB0 Opened Successfully ");

    //------------------------------------Termios-------------------------------------------
    //Setting the Port Settings
    //www.cmrr.umn.edu/~strupp/serial.html#2_3_1
    struct termios SerialPortSettings;
  
    tcgetattr(fd, &SerialPortSettings); // Get the current attributes of the Serial port 
    cfsetispeed(&SerialPortSettings,B57600); // Set Read  Speed as 19200                       
    cfsetospeed(&SerialPortSettings,B57600); // Set Write Speed as 9600                             
    
    /*For experimenting with other port settings*/
    // Even parity (7E1):
    // SerialPortSettings.c_cflag |= PARENB;
    // SerialPortSettings.c_cflag &= ~PARODD;
    // SerialPortSettings.c_cflag &= ~CSTOPB;
    // SerialPortSettings.c_cflag &= ~CSIZE;
    // SerialPortSettings.c_cflag |= CS7; 
  
    // No parity (8N1):
    SerialPortSettings.c_cflag &= ~PARENB;
    SerialPortSettings.c_cflag &= ~CSTOPB;
    SerialPortSettings.c_cflag &= ~CSIZE;
    SerialPortSettings.c_cflag |= CS8;                               
  
    // SerialPortSettings.c_cflag &= ~CRTSCTS;       // No Hardware flow Control                         
    SerialPortSettings.c_cflag |= CREAD | CLOCAL; // Enable receiver,Ignore Modem Control lines        
    
    
    // SerialPortSettings.c_iflag |= (IXON | IXOFF | IXANY);// Enable XON/XOFF flow control both i/p and o/p 
    SerialPortSettings.c_iflag &= ~(IXON | IXOFF | IXANY);// Disable XON/XOFF flow control both i/p and o/p 
    
    //SerialPortSettings.c_iflag &= ~(IXANY); 
    
    // SerialPortSettings.c_iflag &= ~(ICANON | ECHO | ECHOE | ISIG);// raw mode                             
    SerialPortSettings.c_iflag |= (ICANON | ECHO | ECHOE | ISIG);// Cannonical mode                             
  
    SerialPortSettings.c_oflag &= ~OPOST;//No Output Processing
  
    // Setting Time outs
    SerialPortSettings.c_cc[VMIN] = 0; // Read at least 10 characters 21
    SerialPortSettings.c_cc[VTIME] = 0; // Wait indefinetly
  
    if((tcsetattr(fd,TCSANOW,&SerialPortSettings)) != 0) // Set the attributes to the termios structure
          printf("\n  ERROR ! in Setting attributes");
    else
        printf("\n  BaudRate = 57600 \n  StopBits = 1 \n  Parity   = none"); 

    //--------------------------Define ROS Subscribe and Advertise nodes-------------------------------

    sb_imu = nh.subscribe("/mavros/imu/data",10, &windData::windGrab,this);

    pb_merged = nh.advertise<anemometer::ImuConverged>("/mavros/wind_imu", 1);
}

//------------- CALLBACK function to receive item_order data---------------

void windData::windGrab(const sensor_msgs::Imu msg)
{
	ROS_INFO("Hey Im in the Callback!------------------START---------->");  
	//Starts timer between IMU and Anemometer timestamps, looking for lag...
	//double time1 = ros::Time::now().toSec(); 

	ros::Time imu_time = msg.header.stamp;
	double time_imu = imu_time.toSec();
	ROS_INFO("IMU time stamp is %f seconds",time_imu);
	//ROS_INFO("Difference between IMU timestamp and windGrab Callback start time is %f seconds ",time1-time_imu);
  
	//Declare "converged" object that hold both IMU and anemometer data
	anemometer::ImuConverged converged; 

    /*------------------------------- Write data to serial port -----------------------------*/
  	// double t_to_anemo = ros::Time::now().toSec(); //Starts timer as anemometer trigger sent
	//***Start of Try-Catch
	try{

		char write_buffer[2] = {'*'};  // Buffer containing characters to write into port 
 
    	int  bytes_written = 0;    /* Value for storing the number of bytes written to the port */ 
  		bytes_written = write(fd,write_buffer,sizeof(write_buffer)); //use write() to send data to port
   
	    if(bytes_written == -1)
	      { 
	        printf("\n  W\n"); // "W" is an abbreviated Write Error, used for try-catch error checks
	        throw 0;
	      }
                           
        /*Checking data sent out on trigger buffer */                       
	    // printf("\n  ->%s<-written to ttyUSBX\n",write_buffer);       //COMMENTED OUT FOR SPEED
	    // printf("\n  %d Bytes written to ttyUSBX\n", bytes_written);  //COMMENTED OUT FOR SPEED

  
        /*------------------------------- Read data from serial port -----------------------------*/

	  	char read_buffer[22];
	  	int  bytes_read = 0;    // Number of bytes read by the read() system call 
	  	int i = 0;

		int bytes_avail = 0;
		ioctl(fd, FIONREAD, &bytes_avail); //checks if anything on incoming serial line
		printf("\n  bytes on input line is: %d\n", bytes_avail);

		/*Checking the serial line to see if more data is coming
		This portion of code is the reason why this node can't read anemometer data
		faster than about 50hz. It is the usleep() operations that can jam transmission,
		meaning it can block port while parts of the data payload are lost...
		This is not the best way to "wait" for a complete payload of wind data.
		I'll find a better way to accomplish this in the future, perhaps through a
		default setting in the termios port object*/
	  	if(bytes_avail < 21) 
	  	{ 
		    int bytes_avail2 = 0;  
		    printf("\n  S\n"); 
		    usleep(8000); // microseconds at least (a+b) for sonic to take 1 sample
		                   // Data aq. time (a) = 5000 microsec/sample 
		                   // Anemometer process and put on lines (b) = 3000 microsec/sample
		                   // Sonic Anemometer Operators Manual pg 11
		    ioctl(fd, FIONREAD, &bytes_avail2); ///////////
		    printf("\n  bytes on input line [NAPPED] is: %d\n", bytes_avail2); ///////////

		    if(bytes_avail2 < 21){   
		    printf("\n  SS\n");  
		    throw 0;
	    	}
	  	}    
  
  		// Reading Line
  		bytes_read = read(fd,read_buffer,sizeof(read_buffer)); // Read the data 
    	tcflush(fd, TCIFLUSH); //Always flush after reading

  		// Error Checking the Reading Line
  		if(bytes_read == -1){    
      		printf("\n  R\n");
      		throw 0;
  		}
  
		/*Output to user about total time needed to take a wind reading*/
		// double t_from_anemo = ros::Time::now().toSec();
		// ROS_INFO("Time to and from Anemometer is %f seconds",t_from_anemo-t_to_anemo);
		// printf("Read Buffer is: ->%s<-",read_buffer);
		// printf("\n\n  Bytes Rxed -%d", bytes_read); // Print the number of bytes read   //COMMENTED OUT FOR SPEED
		// printf("\n\n  ");                                                               //COMMENTED OUT FOR SPEED

        //---------------------Print on one line------------------------------
  		for(i=0;i<bytes_read;i++)  //printing only the received characters
      		printf("%c",read_buffer[i]);
  			printf("\n +----------------------------------+\n\n\n");    //COMMENTED OUT FOR SPEED


        //---------------Parse out U,V,W,T (with decimal point)----------------
  		/*The decimal point is needed because the data comes in as 1234 
  		and we need it as 12.34 prior to atof() conversion*/ 
        int word_length = 5; //6
        char U[] = {'0','0','0','0','0','0','\0'};
        char V[] = {'0','0','0','0','0','0','\0'};
        char W[] = {'0','0','0','0','0','0','\0'};
        char T[] = {'0','0','0','0','0','0','\0'};

        for(int j=0;j<word_length;j++)
        {
            if(j == 3) //Adds decimal point
            {
              U[j] = '.';
              V[j] = '.';
              W[j] = '.';
              T[j] = '.';
            }
            if(j < 3) //adds 10s and 1s places
            {
              U[j] = read_buffer[j];
              V[j] = read_buffer[j+5]; //7
              W[j] = read_buffer[j+10]; //14
              T[j] = read_buffer[j+15]; //21
            }
            else //adds 10ths and 100ths places
            {
              U[j+1] = read_buffer[j];
              V[j+1] = read_buffer[j+5]; 
              W[j+1] = read_buffer[j+10];
              T[j+1] = read_buffer[j+15];        
            }
        }

		ROS_INFO("U is %.6s m/s",U);
		ROS_INFO("V is %.6s m/s",V);
		ROS_INFO("W is %.6s m/s",W);
		ROS_INFO("T is %.6s deg C",T);

		//Convert char arrays into doubles (X_d) to publish
		double U_d = atof(U);
		double V_d = atof(V);
		double W_d = atof(W);
		double T_d = atof(T);



        //populate converged message object with calculated data
		converged.seq = msg.header.seq;
		converged.stamp = msg.header.stamp;
		converged.frame_id = msg.header.frame_id;
		converged.x_ang_vel = msg.angular_velocity.x;
		converged.y_ang_vel = msg.angular_velocity.y;  
		converged.z_ang_vel = msg.angular_velocity.z;
		converged.angular_velocity_covariance = msg.angular_velocity_covariance;  
		converged.x_lin_acc = msg.linear_acceleration.x;
		converged.y_lin_acc = msg.linear_acceleration.y;
		converged.z_lin_acc = msg.linear_acceleration.z;
		converged.linear_acceleration_covariance = msg.linear_acceleration_covariance;
		converged.wind_U = U_d;  //U_d
		converged.wind_V = V_d;  //V_d
		converged.wind_W = W_d;  //W_d
		converged.temp = T_d;
		
		//Publish out Raw Converged Message Packet
		pb_merged.publish(converged);

	//***End of Try-Catch	
	}catch(int x)
	{
	
	  tcflush(fd, TCIFLUSH); //deletes port so that node can shutdown properly
	}


	/*Additional Debug code to check time lag between Anemometer callback and its trigger */
	//  ROS_INFO("Linear x accel is: %f",converged.x_lin_acc);
  	//  float CbToAne =  t_to_anemo - time1;
	//  ROS_INFO("Time between CB starting Anemometer Trigger Sending is %f seconds",CbToAne);
  	//  double time2 = ros::Time::now().toSec();
	//  ROS_INFO("Delta time for Entire windGrab callback is %f seconds",time2-time1);
	//  ROS_INFO("Entire windGrab callback duration without anemometer is is %f seconds",(time2-time1)-(t_from_anemo-t_to_anemo));
	//  ROS_INFO("Offset between IMU data timestamp and callback completion is %f seconds",time2-time_imu);
	//  ROS_INFO("Offset between IMU data timestamp and when wind was measured is %f seconds",((t_to_anemo-time_imu)+0.01));


  ROS_INFO("Call Back done----------------------------END------->\n");  

}

//--------------------------main program---------------------------------------------
int main(int argc, char** argv)
{

  // initializing ros
  ROS_INFO(" initializing anemometer node");
  ros::init(argc, argv, "anemometer"); //initiating node with unique node name
  
  windData *wind_imu; //pointer to a windData class object
  wind_imu = new windData(); //instantiates memory in the heap

  ROS_INFO(" instantiated object");
  //ros::Rate loop_rate(10); //send message at 10 hz

  while(ros::ok)
  {
    ROS_INFO(" before ros spin()");
    ros::spin();
    ROS_INFO(" after ros spin()");

  }

  delete(wind_imu); //deletes windData object

}
