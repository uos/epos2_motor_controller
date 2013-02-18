// Copyright (C) 2009-2010 Institut de Robòtica i Informàtica Industrial, CSIC-UPC.
// Author Martí Morta Garriga  (mmorta@iri.upc.edu)
// All rights reserved.
//
// This file is part of IRI EPOS2 Driver
// IRI EPOS2 Driver is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#include "Epos2.h"
#include "ctime.h"
//#define DEBUG

// ----------------------------------------------------------------------------
//   CLASS
// ----------------------------------------------------------------------------
//     CONSTRUCTOR
// ----------------------------------------------------------------------------

CEpos2::CEpos2(std::string id)
{
  this->verbose = false;
  this->comm_dev = NULL;
  this->pid = 0xa8b0;
  this->id = id;

  CTime start_time;
  start_time.setFormat(ctf_ms);
  std::stringstream ts_id;
  ts_id << "_" << start_time;

  this->threads = CThreadServer::instance();
  this->target_reached_thread_id = "target_reached_" + id + ts_id.str();
  this->position_marked_thread_id = "position_marked_" + id + ts_id.str();
  this->threads->create_thread(this->target_reached_thread_id);
  this->threads->attach_thread(this->target_reached_thread_id,
                               this->threadTargetReached, this);

  this->events = CEventServer::instance();
  this->target_reached_event_id = "target_reached_" + id + ts_id.str();
  this->position_marked_event_id = "position_marked_" + id + ts_id.str();
  this->stop_marking_event_id = "stop_marking_" + id + ts_id.str();
  this->events->create_event(this->target_reached_event_id);
  this->events->create_event(this->position_marked_event_id);
  this->events->create_event(this->stop_marking_event_id);
  POSITION_MARKED.push_back(this->position_marked_event_id);
  TARGET_REACHED.push_back(this->target_reached_event_id);

}

//     DESTRUCTOR
// ----------------------------------------------------------------------------

CEpos2::~CEpos2()
{
}

// ----------------------------------------------------------------------------
//   OPERATION
// ----------------------------------------------------------------------------
//     INIT
// ----------------------------------------------------------------------------

void CEpos2::init()
{
  p("init()");

  this->openDevice();
  p("readStatusWord()");  
  try{
    this->readStatusWord();
  }catch(CException &e)
  {
    throw CEpos2Exception(_HERE_,"Impossible to read Status Word.\n     Is the controller powered ?");
  }
}

//     CLOSE
// ----------------------------------------------------------------------------

void CEpos2::close()
{
  p("close()");
  this->disableVoltage();

  delete this->comm_dev;

  p("offline");
}

//     P (print for debug) (stringstream)
// ----------------------------------------------------------------------------

void CEpos2::p(const std::stringstream& text)
{
	if(this->verbose) std::cout << "    [EPOS2] " << text.str() << std::endl;
}

//     P (char *)
// ----------------------------------------------------------------------------

void CEpos2::p(const char *text)
{
  if(this->verbose) std::cout << "    [EPOS2] " << text << std::endl;
}

//     GET VERBOSE
// ----------------------------------------------------------------------------

bool CEpos2::getVerbose()
{
  return(this->verbose);
}

//     SET VERBOSE
// ----------------------------------------------------------------------------

void CEpos2::setVerbose(bool verbose)
{
  this->verbose=verbose;
}


//----------------------------------------------------------------------------
//   COMMUNICATION
// ----------------------------------------------------------------------------
//     OPEN DEVICE
// ----------------------------------------------------------------------------

void CEpos2::openDevice()
{
  this->ftdis = CFTDIServer::instance();

  this->comm_dev = NULL;
  std::string desc   = "";
  std::string serial = "";
  std::string name = "";

  if(this->comm_dev!=NULL)
    this->close();

  int num=0;

  try
  {
    ftdis->add_custom_PID(this->pid);

    num = ftdis->get_num_devices();
  }catch(CException &e)
  {
    throw CEpos2Exception(_HERE_,e.what());
  }

  if(num<=0)
    throw CEpos2Exception(_HERE_,"No FTDI devices connected");

  try
  {

    for(int i=0;i<num;i++)
    {
      name = "epos2_";
      desc = this->ftdis->get_description(i);

      if( desc.find("EPOS2") != std::string::npos )
      {
        serial = this->ftdis->get_serial_number(i);
        name += serial;
        this->comm_dev = this->ftdis->get_device(serial);;
        std::stringstream s;
        s << "EPOS2 : " << serial;
        p(s);s.str("");
      }
    }

    TFTDIconfig ftdi_config;

    ftdi_config.baud_rate     = 1000000;
    ftdi_config.word_length   = 8;
    ftdi_config.stop_bits     = 0;
    ftdi_config.parity        = 0;
    ftdi_config.read_timeout  = 10000;
    ftdi_config.write_timeout = 10000;
    ftdi_config.latency_timer = 0;

    this->comm_dev->config(&ftdi_config);

    std::stringstream s;
    s << (*this->comm_dev);
    p(s);

  }catch(CException &e)
  {
    throw CEpos2Exception(_HERE_,e.what());
  }

}

//     READ OBJECT
// ----------------------------------------------------------------------------

int32_t CEpos2::readObject(int16_t index, int8_t subindex)
{
  int32_t result = 0x00000000;
  int16_t req_frame[4];
  uint16_t ans_frame[4];
  int8_t  node_id = 0x00; // TODO attribute of the class

  req_frame[0] = 0x0210;     // header (LEN,OPCODE)
  req_frame[1] = index;      // data
  req_frame[2] = ((0x0000 | node_id) << 8) | subindex; // node_id subindex
  req_frame[3] = 0x0000;     // CRC

  try{
    
    //p("readObject: sendFrame");
    this->sendFrame(req_frame);

    //printf("RF: %.2X %.2X %.2X %.2X\n",req_frame[0],req_frame[1],req_frame[2],req_frame[3]);

    //p("readObject: receiveFrame");
    this->receiveFrame(ans_frame);

    //printf("AF: %.2X %.2X %.2X %.2X\n",ans_frame[0],ans_frame[1],ans_frame[2],ans_frame[3]);

    // if 0x8090, its 16 bit answer else is 32 bit
    if(ans_frame[3]==0x8090)
      result = ans_frame[2];
    else
      result = (ans_frame[3] << 16) | ans_frame[2];

    //printf("result: %d %d -> %d\n",ans_frame[3],ans_frame[2],result);

  }catch(CException &e)
  {
    std::cout << "ERROR: " << e.what() <<std::endl;
    throw CEpos2Exception(_HERE_, "read object error" );
  }

  return result;
}

//     WRITE OBJECT
// ----------------------------------------------------------------------------

int32_t CEpos2::writeObject(int16_t index, int8_t subindex, int32_t data)
{
  int32_t result = 0;
  int16_t req_frame[6]={0,0,0,0,0,0};
  uint16_t ans_frame[40]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
  int8_t node_id = 0x00;

  req_frame[0] = 0x0411;     // header (LEN,OPCODE)
  req_frame[1] = index;      // data
  req_frame[2] = ((0x0000 | node_id) << 8) | subindex;
  req_frame[3] = data & 0x0000FFFF;
  req_frame[4] = data >> 16;
  req_frame[5] = 0x0000;     // checksum

  try{
    this->sendFrame(req_frame);
    this->receiveFrame(ans_frame);

    // if 0x8090, its 16 bit answer else is 32 bit
    if(ans_frame[3]==0x8090)
      result = ans_frame[2];
    else
      result = (ans_frame[3] << 16) | ans_frame[2];

  }catch(CException &e)
  {
    throw CEpos2Exception(_HERE_, "write object error" );
  }

  return result;
}

//     SEND FRAME
// ----------------------------------------------------------------------------

void CEpos2::sendFrame(int16_t *frame)
{
  uint8_t trans_frame[80];                  // transmission frame
  int16_t length = ((frame[0] & 0xFF00) >> 8 ) + 2;   // frame length

  // Add checksum to the frame
  frame[length-1] = this->computeChecksum(frame,length);

  // add SYNC characters (DLE and STX)
  trans_frame[0] = 0x90;
  trans_frame[1] = 0x02;

  // Stuffing
  int8_t i=0, tf_i=2;
  while( i < length )
  {
      // LSB
      trans_frame[tf_i] = frame[i] & 0x00FF;
      if( trans_frame[tf_i] == 0x90 )
      {
        tf_i++;
        trans_frame[tf_i] = 0x90;
      }
      tf_i++;

      // MSB
      trans_frame[tf_i] = (frame[i] & 0xFF00) >> 8;
      if( trans_frame[tf_i] == 0x90 )
      {
        tf_i++;
        trans_frame[tf_i] = 0x90;
      }
      tf_i++;
      i++;
  }

  // write transmission frame
  try{
    //p("sendFrame: write comm");
    this->comm_dev->write(trans_frame, tf_i);
    //printf("bytes written: %d\n",bytes);

  }catch(CFTDIException &e)
  {
    std::cout << e.what() << std::endl;
    throw CEpos2Exception(_HERE_, "write error" );
  }
}

//     RECEIVE FRAME
// ----------------------------------------------------------------------------

void CEpos2::receiveFrame(uint16_t* ans_frame)
{
  // events
  CEventServer *e_s = CEventServer::instance();
  std::list<std::string> data_arrived;
  std::string rx = this->comm_dev->get_rx_event_id();
  data_arrived.push_back(rx);

  // length variables
  int read_desired         = 0;       // length of data that must read
  int read_real            = 0;       // length of data read actually
  int Len                  = 0;       // Len header part in epos2 usb frame
  int read_point           = 0;       // Position of the data read
  int state                = 0;       // state of the parsing state machine
  bool packet_complete     = false;

  // data holders
  uint8_t *read_buffer = NULL;  // frame buffer stuffed
  uint8_t *data        = NULL;  // frame buffer unstuffed
  uint8_t cheksum[2];

  // Read usb data
  try{

    // get data packet
    do{
      e_s->wait_all(data_arrived,100);

      read_desired = this->comm_dev->get_num_data();
      
      if(read_buffer!=NULL)
        delete[] read_buffer;

      read_buffer = new uint8_t[read_desired];

      read_real    = this->comm_dev->read( read_buffer, read_desired );

      if(read_real != read_desired)
        throw CEpos2Exception(_HERE_,"readed data length is not the desired");

      // parsing data
      //printf("%d%",read_real);

      for(int i=0;i<read_real;i++)
      {
        //printf("%.2X.",read_buffer[i]);

        switch (state)
        {
          case 0:
          // no sync
            if(read_buffer[i] == 0x90)
              state = 1;
            else
              state = 0;
            break;
          case 1:
            // sync stx
            if(read_buffer[i] == 0x02)
              state = 2;
            else
              state = 0;
            break;
          case 2:
            // opcode
            state = 3;
           break;
          case 3:
            // len (16 bits)
            Len = read_buffer[i];
            if(data!=NULL)
              delete[] data;
            data = new uint8_t[Len*2];
            read_point = -1;
            state = 4;
            break;
          case 4:
            read_point++;
            data[read_point] = read_buffer[i];
            if(data[read_point]==0x90)
            {
              state = 5;
            }else{
              if(read_point+1 == Len*2)
                state = 6;
              else
                state = 4;
            }
            break;
          case 5:
            // destuffing
              state = 4;
              break;
          case 6:
            // checksum 1
            cheksum[1] = read_buffer[i];
            if(cheksum[1]==0x90){
              state = 8;
            }else{
              state = 7;
            }
            break;
          case 7:
            // checksum 0
            cheksum[0] = read_buffer[i];
            if(cheksum[0]==0x90){
              state = 9;
            }else{
              state = 0;
              packet_complete = true;
            }
            break;
          case 8:
            // destuff checksum 1
            state = 7;
            break;
          case 9:
            // destuff checksum 0
            state = 0;
            packet_complete = true;
            break;
        }
      }
      //printf(" - ");

    }while(!packet_complete);

    //printf("\n");

  }catch(CException &e)
  {
    std::cout << e.what() << std::endl;
    throw CEpos2Exception(_HERE_, "reading error" );
  }

  // parse data
  //printf("AF = ");
  int tf_i = 0;
  for(int i = 0; i < Len; i++)
  {
    ans_frame[i] = 0x0000;
    // LSB to 0x__··
    ans_frame[i] = data[tf_i];
    //printf("[ %.4X ] ",ans_frame[i]);
    tf_i++;
    // MSB to 0x··__
    ans_frame[i] = (data[tf_i]<<8) | ans_frame[i];
    tf_i++;
    //printf("%.4X ",ans_frame[i]);
  }
  //printf(" ");

  if(data!=NULL)
    delete[] data;
  if(read_buffer!=NULL)
    delete[] read_buffer;

}

//     COMPUTE CHECKSUM
// ----------------------------------------------------------------------------

int16_t CEpos2::computeChecksum(int16_t *pDataArray, int16_t numberOfWords)
{
  uint16_t shifter, c;
  uint16_t carry;
  uint16_t CRC = 0;

  //Calculate pDataArray Word by Word
  while(numberOfWords--)
  {
    shifter = 0x8000;                 //Initialize BitX to Bit15
    c = *pDataArray++;                //Copy next DataWord to c
    do
    {
      carry = CRC & 0x8000;    //Check if Bit15 of CRC is set
      CRC <<= 1;               //CRC = CRC * 2
      if(c & shifter) CRC++;   //CRC = CRC + 1, if BitX is set in c
      if(carry) CRC ^= 0x1021; //CRC = CRC XOR G(x), if carry is true
      shifter >>= 1;           //Set BitX to next lower Bit, shifter = shifter/2
    } while(shifter);
  }

  return (int16_t)CRC;
}


//----------------------------------------------------------------------------
//   MANAGEMENT
// ----------------------------------------------------------------------------
//     Get State
// ----------------------------------------------------------------------------

long CEpos2::getState()
{


	long ans = this->readObject(0x6041, 0x00);

  std::stringstream s;
  s << "Estat: " << ans << " /  std::dec= " <<std::dec<< ans;
  p(s);

	// OBTENIR EL NUMERO D'ESTAT
	bool bits[16];
	bits[0]=  (ans & 0x0001);
	bits[1]=  (ans & 0x0002);
	bits[2]=  (ans & 0x0004);
	bits[3]=  (ans & 0x0008);

	bits[4]=  (ans & 0x0010);
	bits[5]=  (ans & 0x0020);
	bits[6]=  (ans & 0x0040);
	bits[7]=  (ans & 0x0080);

	bits[8]=  ans & 0x0100;
	bits[9]=  ans & 0x0200;
	bits[10]= ans & 0x0400;
	bits[11]= ans & 0x0800;

	bits[12]= ans & 0x1000;
	bits[13]= ans & 0x2000;
	bits[14]= ans & 0x4000;
	bits[15]= ans & 0x8000;



  #ifdef DEBUG
	std::cout
	<< bits[15]
	<< bits[14]
	<< bits[13]
	<< bits[12]
	<< bits[11]
	<< bits[10]
	<< bits[9]
	<< bits[8]
	<< bits[7]
	<< bits[6]
	<< bits[5]
	<< bits[4]
	<< bits[3]
	<< bits[2]
	<< bits[1]
	<< bits[0]
	<< std::endl;
  #endif

	if(bits[14]){
		if(bits[4]){
      p("State: Measure Init");
			return(MEASURE_INIT);
		}else{
      p("State: Refresh");
			return(REFRESH);
		}
	}else{
		if(!bits[8]){
      p("State: Start");
			return(START);
		}else{
			if(bits[6]){
        p("State: Switch on disabled");
				return(SWITCH_ON_DISABLED);
			}else{
				if(bits[5]){
					if(bits[4]){
            p("State: Operation Enable");
						return(OPERATION_ENABLE);
					}else{
						if(bits[1]){
              p("State: Switched On");
							return(SWITCH_ON);
						}else{
              p("State: Ready to Switch On");
							return(READY_TO_SWITCH_ON);
						}
					}
				}else{
					if(!bits[3]){
						if(bits[2]){
              p("State: Quick Stop Active");
						return(QUICK_STOP);
						}else{
              p("State: Not Ready to Switch On");
							return(NOT_READY_TO_SWITCH_ON);
						}
					}else{
						if(bits[4]){
              p("State: Fault Reaction Active (Enabled)");
							return(QUICK_STOP_ACTIVE_ENABLE);
						}else{
							if(bits[2]){
                p("State: Fault Reaction Active (Disabled)");
								return(QUICK_STOP_ACTIVE_DISABLE);
							}else{
                p("State: Fault");
								return(FAULT);
							}
						}
					}
				}
			}
		}
	}
	// Error
  std::cout << this->searchErrorDescription( this->readError() ) << std::endl;
	throw CEpos2Exception(_HERE_, "State Error");

}

//     SHUTDOWN (transition)
// ----------------------------------------------------------------------------

void CEpos2::shutdown()
{
  this->writeObject(0x6040, 0x00, 0x06);
}

//     SWITCH ON (transition)
// ----------------------------------------------------------------------------

void CEpos2::switchOn()
{
  this->writeObject(0x6040, 0x00, 0x07);
}

//     DISABLE VOLTAGE (transition)
// ----------------------------------------------------------------------------

void CEpos2::disableVoltage()
{
  this->writeObject(0x6040, 0x00, 0x00);
}

//     QUICK STOP (transition)
// ----------------------------------------------------------------------------

void CEpos2::quickStop()
{
  this->writeObject(0x6040, 0x00, 0x02);
}

//     DISABLE OPERATION (transition)
// ----------------------------------------------------------------------------

void CEpos2::disableOperation()
{
  this->writeObject(0x6040, 0x00, 0x07);
}

//     ENABLE OPERATION (transition)
// ----------------------------------------------------------------------------

void CEpos2::enableOperation()
{
  this->writeObject(0x6040, 0x00, 0x0F);
}

//     FAULT RESET (transition)
// ----------------------------------------------------------------------------

void CEpos2::faultReset()
{
  this->writeObject(0x6040, 0x00, 0x80);
}

//----------------------------------------------------------------------------
//   OPERATION MODES
// ----------------------------------------------------------------------------
//     GET OPERATION MODE
// ----------------------------------------------------------------------------

long CEpos2::getOperationMode()
{
  long ans = this->readObject(0x6061, 0x00);

	// Rectificacio
  /// \todo veure si això fa falta
  //ans = this->getNegativeLong(ans);

  std::stringstream s;
  s << this->getOpModeDescription(ans);
  p(s);

	return(ans);
}

//     GET OPERATION MODE DESCRIPTION
// ----------------------------------------------------------------------------

std::string CEpos2::getOpModeDescription(long opmode)
{

	std::stringstream s;
	std::string       name;

	switch(opmode){
		case VELOCITY:
			name="Velocity";
		break;
		case POSITION:
			name="Position";
		break;
    case PROFILE_POSITION:
			name="Profile Position";
			break;
    case PROFILE_VELOCITY:
			name="Profile Velocity";
			break;
    case INTERPOLATED_PROFILE_POSITION:
			name="Interpolated Profile Position";
			break;
    case HOMING:
			name="Homing";
			break;
	}

  s << "Operation Mode: " << name;
  this->p(s);

	return(name);

}

//     SET OPERATION MODE
// ----------------------------------------------------------------------------

void CEpos2::setOperationMode(long opmode)
{
	std::stringstream s;

	if(
	  opmode != VELOCITY &&
	  opmode != POSITION &&
	  opmode != PROFILE_VELOCITY &&
	  opmode != PROFILE_POSITION &&
	  opmode != INTERPOLATED_PROFILE_POSITION &&
	  opmode != HOMING
	  ){
		if(opmode == NO_OPERATION){
      std::cout << "    [EPOS2] Operation Mode not changed: " << opmode << std::endl;
		}else{
			std::cout << "    [EPOS2] ! ERROR: MODE NOT KNOWN:" << opmode
          << " ACTUAL MODE: " << this->getOperationMode() << std::endl;
		}
	  }else{
      this->writeObject(0x6060, 0x00,opmode);

      s << "Operation Mode: " << opmode;
      this->p(s);
	  }
}

//     ENABLE CONTROLLER
// ----------------------------------------------------------------------------

void CEpos2::enableController()
{
	int estat=0,timeout=0;
  bool controller_connected = false;

  estat = this->getState();

  while( !controller_connected && timeout<10 )
  {
    switch(estat)
    {
      case 0:
        // FAULT
        this->faultReset();
        timeout++;
        break;
      case 1:
        // START
        break;
      case 2:
        // NOT_READY_TO_SWITCH_ON
        break;
      case 3:
        // SWITCH_ON_DISABLED
        timeout++;
        this->shutdown();
        break;
      case 4:
        // READY_TO_SWITCH_ON
        this->switchOn();
        break;
      case 5:
        // SWITCH_ON
        controller_connected = true;
        break;
      case 6:
        // REFRESH
        break;
      case 7:
        // MEASURE_INIT
        break;
      case 8:
        // OPERATION_ENABLE
        this->disableOperation();
        break;
      case 9:
        // QUICK_STOP
        this->disableVoltage();
        break;
      case 10:
        // QUICK_STOP_ACTIVE_DISABLE
        break;
      case 11:
        // QUICK_STOP_ACTIVE_ENABLE
        break;
    }
    estat = this->getState();
  }

}

//     ENABLE MOTOR
// ----------------------------------------------------------------------------

void CEpos2::enableMotor(long opmode)
{
	int estat;
  std::stringstream s;

  estat = this->getState();

	if( estat == SWITCH_ON )
	{
    this->enableOperation();
	}
  this->p("Operation Enable");

	if( opmode != NO_OPERATION )
	{
    if( this->getOperationMode() != opmode )
		{
      this->setOperationMode(opmode);
		}
    this->p("Motor Ready");
	}
}


//     IS TARGET REACHED ? (Shared between modes)
// ----------------------------------------------------------------------------

bool CEpos2::isTargetReached()
{

  long ans = this->readObject(0x6041, 0x00);
  std::stringstream s;

	//s << "Estat: std::hex=" <<std::hex<< ans << " /  std::dec=" <<std::dec << ans;
  //this->p(s);
	// OBTENIR EL NUMERO D'ESTAT

	// bit10 = 0 , not reached
	// bit10 = 1 , reached

	return((bool)(ans & 0x0400));
}

//----------------------------------------------------------------------------
//   MODE VELOCITY
// ----------------------------------------------------------------------------
//     GET TARGET VELOCITY
// ----------------------------------------------------------------------------

long CEpos2::getTargetVelocity()
{
  long ans = this->readObject(0x206B, 0x00);
  std::stringstream s;

	s << "Velocity: " <<std::hex<< ans << " / " <<std::dec<< ans << std::endl;
  this->p(s);

	return(ans);
}

//     SET TARGET VELOCITY
// ----------------------------------------------------------------------------

void CEpos2::setTargetVelocity(long velocity)
{
  this->writeObject(0x206B, 0x00,velocity);
}

//     START VELOCITY
// ----------------------------------------------------------------------------

void CEpos2::startVelocity()
{
  this->writeObject(0x6040, 0x00, 0x010f);
  this->p("Mode Velocity Started");
}

//     STOP VELOCITY
// ----------------------------------------------------------------------------

void CEpos2::stopVelocity()
{
  // just velocity command = 0
  this->writeObject(0x206B, 0x00,0x0000);
  this->p("Mode Velocity Stopped");
}

//----------------------------------------------------------------------------
//   MODE PROFILE VELOCITY
// ----------------------------------------------------------------------------
//     GET TARGET PROFILE VELOCITY
// ----------------------------------------------------------------------------

long CEpos2::getTargetProfileVelocity()
{
  std::stringstream s;

  long ans = this->readObject(0x60FF, 0x00);
  s << "Profile Velocity: " <<std::hex<< ans << " / " <<std::dec<< ans << std::endl;
  this->p(s);

	return(ans);
}

//     SET TARGET PROFILE VELOCITY
// ----------------------------------------------------------------------------

void CEpos2::setTargetProfileVelocity(long velocity)
{
  this->writeObject(0x60FF, 0x00, velocity);
}

//     LISTEN PROFILE VELOCITY
// ----------------------------------------------------------------------------

void* CEpos2::listenProfileVelocity(void* params)
{
	CEpos2 *controller=(CEpos2*)params;

	while(!controller->isTargetReached()){

		// GET information
		if(controller->verbose) controller->getMovementInfo();
	}
  if(controller->verbose) std::cout << "\n";

	controller->p("Target Velocity Reached");;

	pthread_exit(NULL);
}

//     START PROFILE VELOCITY
// ----------------------------------------------------------------------------

void CEpos2::startProfileVelocity()
{
	int intmode = 0x000F;

  this->writeObject(0x6040, 0x00,intmode);

	const char *threadprofilevelocity = "tprofilevelocity";

	threads=CThreadServer::instance();
	threads->create_thread(threadprofilevelocity);
	threads->attach_thread(threadprofilevelocity,listenProfileVelocity,this);
	threads->start_thread(threadprofilevelocity);
	threads->end_thread(threadprofilevelocity);
	threads->delete_thread(threadprofilevelocity);
}

//     STOP PROFILE VELOCITY
// ----------------------------------------------------------------------------

void CEpos2::stopProfileVelocity()
{
	int intmode = 0x010F;
  this->writeObject(0x6040, 0x00,intmode);
}

//----------------------------------------------------------------------------
//   MODE PROFILE POSITION
// ----------------------------------------------------------------------------
//     GET TARGET PROFILE POSITION
// ----------------------------------------------------------------------------

long CEpos2::getTargetProfilePosition()
{
  std::stringstream s;

  long ans = this->readObject(0x607A, 0x00);

	s << "Profile Target Position: " <<std::hex<< ans << " / " <<std::dec<< ans << std::endl;
  this->p(s);

	return(ans);
}

//     SET TARGET PROFILE POSITION
// ----------------------------------------------------------------------------

void CEpos2::setTargetProfilePosition(long position)
{
  this->writeObject(0x607A, 0x00,position);
}

//     LISTEN PROFILE POSITION
// ----------------------------------------------------------------------------

void* CEpos2::listenProfilePosition(void* params)
{
	CEpos2 *controller=(CEpos2*)params;

  if(controller->verbose)
	{
		while(!controller->isTargetReached()){

			// GET information
			controller->getMovementInfo();
		}
	}else
	{
		usleep(1000);
	}

  if(controller->verbose) std::cout << "\n";

  controller->p(" Target Reached");

	pthread_exit(NULL);
}

// 0 halt, 1 abs, 2 rel
void CEpos2::startProfilePosition(epos_posmodes mode, bool blocking, bool wait, bool new_point)
 {

  int halt        = mode==HALT ?      0x0100 : 0x0000;
  int rel         = mode==RELATIVE ?  0x0040 : 0x0000;
  int nowait      = !wait ?           0x0020 : 0x0000;
  int newsetpoint = new_point ?       0x0010 : 0x0000;

  int intmode = 0x000F | halt | rel | nowait | newsetpoint;

  this->writeObject(0x6040, 0x00,intmode);

  if( !blocking ){
    const char *threadprofileposition= "tprofileposition";
    threads = CThreadServer::instance();

    threads->create_thread(threadprofileposition);
    threads->attach_thread(threadprofileposition,listenProfilePosition,this);
    threads->start_thread(threadprofileposition);
    threads->end_thread(threadprofileposition);
    threads->delete_thread(threadprofileposition);
  }else{

    while( !this->isTargetReached() )
    {
      if(this->verbose) this->getMovementInfo();
      else usleep(1000);
    }
    if(this->verbose) std::cout << "\n";
    this->p("Target Reached");
  }

}



// Current

long CEpos2::getTargetCurrent(){return 1;}

void CEpos2::setTargetCurrent(long current){}

void CEpos2::startCurrent(){}

void CEpos2::stopCurrent(){}



//----------------------------------------------------------------------------
//   CONTROL PARAMETERS
// ----------------------------------------------------------------------------

long CEpos2::getCurrentPGain()
{
  std::stringstream s;

  long ans = this->readObject(0x60F6, 0x01);

	s << "Current P Gain: " <<std::hex<< ans << " / " <<std::dec<< ans << std::endl;
  this->p(s);
	return(ans);

}

void CEpos2::setCurrentPGain(long gain)
{

	if(gain > 32767){
		gain=32767;

		std::cout << "    [EPOS2] Gain > 32767! -> gain=32767" << std::endl;

	}else if(gain < 0) {
		gain = 0;

    std::cout << "    [EPOS2] Gain < 0! -> gain=0" << std::endl;

	}
  this->writeObject(0x60F6, 0x01,gain);
}

long CEpos2::getCurrentIGain()
{
  std::stringstream s;
  long ans = this->readObject(0x60F6, 0x02);

	s << "Current I Gain: " <<std::hex<< ans << " / " <<std::dec<< ans << std::endl;
  this->p(s);
	return(ans);

}

void CEpos2::setCurrentIGain(long gain)
{
	if(gain > 32767){
		gain=32767;

    std::cout << "    [EPOS2] Gain > 32767! -> gain=32767" << std::endl;

	}else if(gain < 0) {
		gain = 0;

    std::cout << "    [EPOS2] Gain < 0! -> gain=0" << std::endl;

	}
  this->writeObject(0x60F6, 0x02,gain);
}

// Velocity

long CEpos2::getVelocityPGain()
{
  std::stringstream s;
  long ans = this->readObject(0x60F9, 0x01);

	s << "Velocity P Gain: " <<std::hex<< ans << " / " <<std::dec<< ans << std::endl;
  this->p(s);
	return(ans);

}

void CEpos2::setVelocityPGain(long gain)
{
	if(gain > 32767){
		gain=32767;

    std::cout << "    [EPOS2] Gain > 32767! -> gain=32767" << std::endl;

	}else if(gain < 0) {
		gain = 0;

    std::cout << "    [EPOS2] Gain < 0! -> gain=0" << std::endl;

	}
  this->writeObject(0x60F9, 0x01,gain);
}

long CEpos2::getVelocityIGain()
{
  std::stringstream s;

  long ans = this->readObject(0x60F9, 0x02);

	s << "Velocity I Gain: " <<std::hex<< ans << " / " <<std::dec<< ans << std::endl;
  this->p(s);
	return(ans);

}

void CEpos2::setVelocityIGain(long gain)
{
	if(gain > 32767){
		gain=32767;

    std::cout << "    [EPOS2] Gain > 32767! -> gain=32767" << std::endl;

	}else if(gain < 0) {
		gain = 0;

    std::cout << "    [EPOS2] Gain < 0! -> gain=0" << std::endl;

	}
  this->writeObject(0x60F9, 0x03,gain);
}

long CEpos2::getVelocitySetPointFactorPGain()
{

  std::stringstream s;
  long ans = this->readObject(0x60F9, 0x03);

	s << "Velocity I Gain: " <<std::hex<< ans << " / " <<std::dec<< ans << std::endl;
  this->p(s);
	return(ans);

}

void CEpos2::setVelocitySetPointFactorPGain(long gain)
{
	if(gain > 32767){
		gain=32767;

    std::cout << "    [EPOS2] Gain > 32767! -> gain=32767" << std::endl;

	}else if(gain < 0) {
		gain = 0;

    std::cout << "    [EPOS2] Gain < 0! -> gain=0" << std::endl;

	}
  this->writeObject(0x60F9, 0x03,gain);
}

// Position

long CEpos2::getPositionPGain()
{

  std::stringstream s;
  long ans = this->readObject(0x60FB, 0x01);

	s << "Velocity I Gain: " <<std::hex<< ans << " / " <<std::dec<< ans << std::endl;
  this->p(s);
	return(ans);

}

void CEpos2::setPositionPGain(long gain)
{
	if(gain > 32767){
		gain=32767;

    std::cout << "    [EPOS2] Gain > 32767! -> gain=32767" << std::endl;

	}else if(gain < 0) {
		gain = 0;

    std::cout << "    [EPOS2] Gain < 0! -> gain=0" << std::endl;

	}
  this->writeObject(0x60FB, 0x01,gain);
}

long CEpos2::getPositionIGain()
{

  std::stringstream s;
  long ans = this->readObject(0x60FB, 0x02);

	s << "Velocity I Gain: " <<std::hex<< ans << " / " <<std::dec<< ans << std::endl;
  this->p(s);
	return(ans);

}

void CEpos2::setPositionIGain(long gain)
{
	if(gain > 32767){
		gain=32767;

    std::cout << "    [EPOS2] Gain > 32767! -> gain=32767" << std::endl;

	}else if(gain < 0) {
		gain = 0;

    std::cout << "    [EPOS2] Gain < 0! -> gain=0" << std::endl;

	}
  this->writeObject(0x60FB, 0x02,gain);
}

long CEpos2::getPositionDGain()
{

  std::stringstream s;
  long ans = this->readObject(0x60FB, 0x03);

	s << "Velocity I Gain: " <<std::hex<< ans << " / " <<std::dec<< ans << std::endl;
  this->p(s);
	return(ans);

}

void CEpos2::setPositionDGain(long gain)
{
	if(gain > 32767){
		gain=32767;

    std::cout << "    [EPOS2] Gain > 32767! -> gain=32767" << std::endl;

	}else if(gain < 0) {
		gain = 0;

    std::cout << "    [EPOS2] Gain < 0! -> gain=0" << std::endl;

	}
  this->writeObject(0x60FB, 0x03,gain);
}

long CEpos2::getPositionVFFGain()
{

  std::stringstream s;
  long ans = this->readObject(0x60FB, 0x04);

	s << "Velocity I Gain: " <<std::hex<< ans << " / " <<std::dec<< ans << std::endl;
  this->p(s);
	return(ans);

}

void CEpos2::setPositionVFFGain(long gain)
{
	if(gain > 65535){
		gain=65535;

    std::cout << "    [EPOS2] Gain > 65535! -> gain=65535" << std::endl;

	}else if(gain < 0) {
		gain = 0;

    std::cout << "    [EPOS2] Gain < 0! -> gain=0" << std::endl;

	}
  this->writeObject(0x60FB, 0x04,gain);
}

long CEpos2::getPositionAFFGain()
{

  std::stringstream s;
  long ans = this->readObject(0x60FB, 0x05);

	s << "Velocity I Gain: " <<std::hex<< ans << " / " <<std::dec<< ans << std::endl;
  this->p(s);
	return(ans);

}

void CEpos2::setPositionAFFGain(long gain)
{
	if(gain > 65535){
		gain=65535;

    std::cout << "    [EPOS2] Gain > 65535! -> gain=65535" << std::endl;

	}else if(gain < 0) {
		gain = 0;

    std::cout << "    [EPOS2] Gain < 0! -> gain=0" << std::endl;

	}
  this->writeObject(0x60FB, 0x05,gain);
}

void CEpos2::getControlParameters(long &cp,long &ci,long &vp,long &vi,
                                  long &vspf, long &pp,long &pi,long &pd,
                                  long &pv,long &pa)
{
  cp = this->getCurrentPGain();
  ci = this->getCurrentIGain();
  vp = this->getVelocityPGain();
  vi = this->getVelocityIGain();
  vspf = this->getVelocitySetPointFactorPGain();
  pp = this->getPositionPGain();
  pi = this->getPositionIGain();
  pd = this->getPositionDGain();
  pv = this->getPositionVFFGain();
  pa = this->getPositionAFFGain();

  if(this->verbose) this->printControlParameters(cp,ci,vp,vi,vspf,pp,pi,pd,pv,pa);

}

void CEpos2::setControlParameters(long cp,long ci,long vp,long vi,long vspf,
                                  long pp,long pi,long pd,long pv,long pa)
{
  this->setCurrentPGain(cp);
  this->setCurrentIGain(ci);
  this->setVelocityPGain(vp);
  this->setVelocityIGain(vi);
  this->setVelocitySetPointFactorPGain(vspf);
  this->setPositionPGain(pp);
  this->setPositionIGain(pi);
  this->setPositionDGain(pd);
  this->setPositionVFFGain(pv);
  this->setPositionAFFGain(pa);


  this->getControlParameters(cp,ci,vp,vi,vspf,pp,pi,pd,pv,pa);

}

void CEpos2::printControlParameters(long cp,long ci,long vp,long vi,long vspf,
                                    long pp,long pi,long pd,long pv,long pa)
{
  std::cout << "    [EPOS2] Control Parameters:" << std::endl;
  std::cout << "    [EPOS2] Current:  P = " << cp << "  I = " << ci << std::endl;
  std::cout << "    [EPOS2] Velocity: P = " << vp << "  I = " << vi << "	SetPointFactorP = " << vspf << std::endl;
  std::cout << "    [EPOS2] Position: P = " << pp << "  I = " << pi << "	D = "<< pd << std::endl;
  std::cout << "    [EPOS2]           Vff = " << pv << "  Aff = " << pa << std::endl;
}

//----------------------------------------------------------------------------
//   PROFILE PARAMETERS
// ----------------------------------------------------------------------------

long CEpos2::getProfileVelocity(void)
{
  std::stringstream s;
  long ans = this->readObject(0x6081, 0x00);

	s << "Profile Velocity: "
      << std::hex << ans << " / " << std::dec << ans << " [rpm]";
  this->p(s);
	return(ans);
}

void CEpos2::setProfileVelocity(long velocity)
{
  this->writeObject(0x6081, 0x00,velocity);
}

long CEpos2::getProfileMaxVelocity(void)
{
  std::stringstream s;
  long ans = this->readObject(0x607F, 0x00);

	s << "Profile Max Velocity: "
      <<std::hex<< ans << " / " <<std::dec<< ans << " [rpm]";
  this->p(s);
	return(ans);
}

void CEpos2::setProfileMaxVelocity(long velocity)
{
  this->writeObject(0x607F, 0x00,velocity);
}

long CEpos2::getProfileAcceleration(void)
{
  std::stringstream s;
  long ans = this->readObject(0x6083, 0x00);

	s << "Profile Acceleration: "
      <<std::hex<< ans << " / " <<std::dec<< ans << " [rpm/s]";
  this->p(s);
	return(ans);
}

void CEpos2::setProfileAcceleration(long acceleration)
{
  this->writeObject(0x6083, 0x00,acceleration);
}

long CEpos2::getProfileDeceleration(void)
{
  std::stringstream s;
  long ans = this->readObject(0x6084, 0x00);

	s << "Profile deceleration: "
      <<std::hex<< ans << " / " <<std::dec<< ans << " [rpm/s]";
  this->p(s);
	return(ans);
}

void CEpos2::setProfileDeceleration(long deceleration)
{
  this->writeObject(0x6084, 0x00,deceleration);
}

long CEpos2::getProfileQuickStopDecel(void)
{
  std::stringstream s;
  long ans = this->readObject(0x6085, 0x00);

	s << "Profile Quick Stop deceleration: "
      <<std::hex<< ans << " / " <<std::dec<< ans << " [rpm/s]";
  this->p(s);
	return(ans);
}

void CEpos2::setProfileQuickStopDecel(long deceleration)
{
  this->writeObject(0x6085, 0x00,deceleration);
}

long CEpos2::getProfileType(void)
{
  std::stringstream s;
  long ans = this->readObject(0x6086, 0x00);

	s << "Profile Type: "
      <<std::hex<< ans << " / " <<std::dec<< ans;
  if( ans == 0 )
    s << " trapezoidal";
  else
    s << " sinusoidal";
  this->p(s);
	return(ans);
}

void CEpos2::setProfileType(long type)
{
  this->writeObject(0x6086, 0x00,type);
}

long CEpos2::getMaxAcceleration(void)
{
  std::stringstream s;
  long ans = this->readObject(0x60C5, 0x00);

	s << "Profile Max Acceleration: " <<std::hex<< ans << " / " <<std::dec<< ans;
  this->p(s);
	return(ans);
}

void CEpos2::setMaxAcceleration(long max_acceleration)
{
  this->writeObject(0x60C5, 0x00,max_acceleration);
}

void CEpos2::getProfileData(long &vel,long &maxvel,long &acc,long &dec,
                            long &qsdec, long &maxacc, long &type)
{
  vel    = this->getProfileVelocity();
  maxvel = this->getProfileMaxVelocity();
  acc    = this->getProfileAcceleration();
  dec    = this->getProfileDeceleration();
  qsdec  = this->getProfileQuickStopDecel();
  maxacc = this->getMaxAcceleration();
  type   = this->getProfileType();
}

void CEpos2::setProfileData(long vel,long maxvel,long acc,long dec,
                            long qsdec,long maxacc,long type)
{
  this->setProfileVelocity(vel);
  this->setProfileMaxVelocity(maxvel);
  this->setProfileAcceleration(acc);
  this->setProfileDeceleration(dec);
  this->setProfileQuickStopDecel(qsdec);
  this->setMaxAcceleration(maxacc);
  this->setProfileType(type);

  long v,m,a,d,q,ma,t;
  this->getProfileData(v,m,a,d,q,ma,t);
}

//----------------------------------------------------------------------------
//   READ INFORMATION
// ----------------------------------------------------------------------------

long CEpos2::readVelocity()
{
  std::stringstream s;
  long ans = this->readObject(0x2028, 0x00);

	s << "Velocity: " <<std::hex<< ans << " / " <<std::dec<< ans << std::endl;
  this->p(s);
	return(ans);
}

long CEpos2::readVelocitySensorActual()
{
  std::stringstream s;
  long ans = this->readObject(0x6069, 0x00);

	s << "Velocity Sensor Actual: " <<std::hex<< ans << " / "
      <<std::dec<< ans << std::endl;
  this->p(s);
	return(ans);
}

long CEpos2::readVelocityDemand()
{
  std::stringstream s;
  long ans = this->readObject(0x606B, 0x00);

	s << "Velocity Demand: " <<std::hex<< ans << " / "
      <<std::dec<< ans << std::endl;
  this-> p(s);
	return(ans);
}

long CEpos2::readVelocityActual	()
{
  std::stringstream s;
  long ans = this->readObject(0x606C, 0x00);

	s << "Velocity Actual: " <<std::hex<< ans << " / "
      <<std::dec<< ans << std::endl;
  this->p(s);
	return(ans);

}

long CEpos2::readCurrent()
{
  std::stringstream s;
  long ans = this->readObject(0x6078, 0x00);

  ans=this->getNegativeLong(ans);

	s << "Current: " <<std::hex<< ans << " / " <<std::dec<< ans << std::endl;
  this->p(s);
	return(ans);
}

long CEpos2::readCurrentAveraged()
{
  std::stringstream s;
  long ans = this->readObject(0x2027, 0x00);

  ans=this->getNegativeLong(ans);

	s << "Current Averaged: " <<std::hex<< ans
      << " / " <<std::dec<< ans << std::endl;
  this->p(s);
	return(ans);
}

long CEpos2::readCurrentDemanded()
{
  std::stringstream s;
  long ans = this->readObject(0x2031, 0x00);

	s << "Current Demanded: " <<std::hex<< ans
      << " / " <<std::dec<< ans << std::endl;
  this->p(s);
	return(ans);
}

int32_t CEpos2::readPosition()
{
  std::stringstream s;
  int32_t ans = this->readObject(0x6064, 0x00);
  // printf("readposition %d\n",ans);

  s << "Pos: " <<std::hex<< ans << " / " <<std::dec<< ans;
  this->p(s);

	return(ans);
}

long CEpos2::readStatusWord()
{
  std::stringstream s;
  p("readStatusWord: read Object");
  long ans = this->readObject(0x6041, 0x00);

	s << "StatusWord: " <<std::hex<< ans << " / " <<std::dec<< ans;
  this->p(s);
	return(ans);
}

long CEpos2::readEncoderCounter()
{

  std::stringstream s;
  long ans = this->readObject(0x2020, 0x00);

	s << "Encoder Counter: " <<std::hex<< ans << " / " <<std::dec<< ans;
  this->p(s);
	return(ans);
}

long CEpos2::readEncoderCounterAtIndexPulse()
{

  std::stringstream s;
  long ans = this->readObject(0x2021, 0x00);

	s << "Encoder Counter at Index Pulse: "
      <<std::hex<< ans << " / " <<std::dec<< ans;
  this->p(s);
	return(ans);
}

long CEpos2::readHallsensorPattern()
{

  std::stringstream s;
  long ans = this->readObject(0x2022, 0x00);

	s << "Hallsensor Pattern: " <<std::hex<< ans << " / " <<std::dec<< ans;
  this->p(s);
	return(ans);
}

long CEpos2::readFollowingError()
{

  std::stringstream s;
  long ans = this->readObject(0x20F4, 0x00);

	s << "Following Error: " <<std::hex<< ans << " / " <<std::dec<< ans;
  this->p(s);
	return(ans);
}

void CEpos2::getMovementInfo()
{
	long vel_actual,vel_avg,vel_demand;
	int cur_actual,cur_avg,cur_demand;
	long pos;
	bool verbose_status;

  verbose_status = this->verbose;
  this->setVerbose(false);
  vel_actual = this->readVelocityActual();
  vel_avg    = this->readVelocity();
  vel_demand = this->readVelocityDemand();

  cur_actual = this->readCurrent();
  cur_avg    = this->readCurrentAveraged();
  cur_demand = this->readCurrentDemanded();

  pos        = this->readPosition();
  this->setVerbose(verbose_status);

	printf("\r    [EPOS2] p: %ld v: %ld vavg: %ld vd: %ld c: %d cavg: %d cd: %d                   ",pos,vel_actual,vel_avg,vel_demand,cur_actual,cur_avg,cur_demand);
		fflush(stdout);

}

//----------------------------------------------------------------------------
//   ERRORS
// ----------------------------------------------------------------------------

char CEpos2::readError()
{
	char error_num=0;
  std::stringstream s;
  long ans = this->readObject(0x1001, 0x00);

	bool bits[8];
	bits[0]=  (ans & 0x0001);
	bits[1]=  (ans & 0x0002);
	bits[2]=  (ans & 0x0004);
	bits[3]=  (ans & 0x0008);

	bits[4]=  (ans & 0x0010);
	bits[5]=  (ans & 0x0020);
	bits[7]=  (ans & 0x0080);

	if(bits[7]) error_num=6; // Motion Error
	if(bits[5]) error_num=5; // Device profile specific
	if(bits[4]) error_num=4; // Communication Error

	if(bits[3]) error_num=3; // Temperature Error
	if(bits[2]) error_num=2; // Voltage Error
	if(bits[1]) error_num=1; // Current Error
	if(bits[0]) error_num=0; // Generic Error

  s << "Error: "<< error_num << " " << this->error_names[(unsigned char)error_num] <<
      " Value: 0x" <<std::hex<< ans << " , " <<std::dec<< ans;
  p(s);

	return(error_num);
}

void CEpos2::readErrorHistory(long *error[5])
{
	std::string error_des;

  long number_errors = this->readObject(0x1003, 0x00);
  std::cout << "  [EPOS2-ERROR] Number of Errors: " << number_errors << std::endl;

	// Read Errors
	for(int i=1;i<=number_errors;i++){
    long ans = this->readObject(0x1003, i);
		error[i] = &ans;
		error_des = this->searchErrorDescription(ans);

		std::cout << "  [EPOS2-ERROR] id: " << i << " : " << std::hex <<"0x"<< ans << " = " << error_des << std::endl;
	}
}

std::string CEpos2::searchErrorDescription(long error_code)
{
	int j=0;
	bool found = false;
  std::stringstream s;

  // error_codes length = 34

  while( !found && j < 34 ){
    if( error_code == this->error_codes[j] ){
			found = true;

      s << "Error Description "<< this->error_descriptions[j] << std::endl;
      p(s);
      return this->error_descriptions[j];

		}else{
			j++;
		}
	}
	if(!found) return "No Description for this error";
	else       return "Error Description";
}

//----------------------------------------------------------------------------
//   VERSIONS
// ----------------------------------------------------------------------------

long CEpos2::readVersionSoftware()
{

  std::stringstream s;
  long ans = this->readObject(0x2003, 0x01);

	s << "Version Software: " <<std::hex<< ans << " / " <<std::dec<< ans << std::endl;
  this->p(s);
	return(ans);
}

long CEpos2::readVersionHardware()
{

  std::stringstream s;
  long ans = this->readObject(0x2003, 0x02);

	s << "Version Hardware: " <<std::hex<< ans << " / " <<std::dec<< ans << std::endl;
  this->p(s);
	return(ans);
}



	// SENSOR CONFIGURATION

long CEpos2::getEncoderPulseNumber(){return 1;}

void CEpos2::setEncoderPulseNumber(long pulses)
{}

long CEpos2::getEncoderType()
{return 1;}

void CEpos2::setEncoderType(long type)
{}

long CEpos2::getEncoderPolarity()
{return 1;}

void CEpos2::setEncoderPolarity(long polarity)
{}

void CEpos2::getEncoderParameters(long &pulses, long &type, long &polarity)
{}

void CEpos2::setEncoderParameters(long pulses, long type, long polarity)
{}

// Motor

long CEpos2::getMotorType(){return 1;}

void CEpos2::setMotorType(long type){}

long CEpos2::getMotorContinuousCurrentLimit(){return 1;}

void CEpos2::setMotorContinuousCurrentLimit(long current_mA){}

long CEpos2::getMotorOutputCurrentLimit(){return 1;}

void CEpos2::setMotorOutputCurrentLimit(long current_mA){}

long CEpos2::getMotorPolePairNumber(){return 1;}

void CEpos2::setMotorPolePairNumber(char pole_pairs){}

long CEpos2::getThermalTimeCtWinding(){return 1;}

void CEpos2::setThermalTimeCtWinding(long time_ds){}

//----------------------------------------------------------------------------
//   UTILITIES
// ----------------------------------------------------------------------------

long CEpos2::getMaxFollowingError()
{

  std::stringstream s;
  long ans = this->readObject(0x6065, 0x00);

	s << "Max Following Error: " <<std::hex<< ans << " / " <<std::dec<< ans << std::endl;
  this->p(s);
	return(ans);

}

void CEpos2::setMaxFollowingError(long error)
{
  this->writeObject(0x6065, 0x00,error);
}

long CEpos2::getMinPositionLimit	(){

  std::stringstream s;
  long ans = this->readObject(0x607D, 0x01);

	s << "Min Position Limit: " <<std::hex<< ans << " / " <<std::dec<< ans << std::endl;
  this->p(s);
	return(ans);

}

void CEpos2::setMinPositionLimit(long limit)
{
  this->writeObject(0x607D, 0x01,limit);
}


long CEpos2::getMaxPositionLimit	(){

  std::stringstream s;
  long ans = this->readObject(0x607D, 0x02);

	s << "Max Position Limit: " <<std::hex<< ans << " / " <<std::dec<< ans << std::endl;
  this->p(s);
	return(ans);

}

void CEpos2::setMaxPositionLimit(long limit)
{
  this->writeObject(0x607D, 0x02,limit);
}

void CEpos2::disablePositionLimits(void)
{
	// min
	// -2147483647-1
  this->writeObject(0x607D, 0x01, -2147483647-1);
	// max
  this->writeObject(0x607D, 0x02, 2147483647);
}

long CEpos2::getPositionWindow(){return 1;}

void CEpos2::setPositionWindow(long window_qc){}

long CEpos2::getPositionWindowTime(){return 1;}

void CEpos2::setPositionWindowTime(long time_ms){}

long CEpos2::getVelocityWindow(){return 1;}

void CEpos2::setVelocityWindow(long window_rm){}

long CEpos2::getVelocityWindowTime(){return 1;}

void CEpos2::setVelocityWindowTime(long time_ms){}

long CEpos2::getPositionNotationIndex(){return 1;}

void CEpos2::setPositionNotationIndex(long notation){}

long CEpos2::getVelocityNotationIndex(){return 1;}

void CEpos2::setVelocityNotationIndex(long notation){}

long CEpos2::getAccelerationNotationIndex(){return 1;}

void CEpos2::setAccelerationNotationIndex(long notation){}

long CEpos2::getPositionDimensionIndex(){return 1;}

void CEpos2::setPositionDimensionIndex(long Dimension){}

long CEpos2::getVelocityDimensionIndex(){return 1;}

void CEpos2::setVelocityDimensionIndex(long Dimension){}

long CEpos2::getAccelerationDimensionIndex(){return 1;}

void CEpos2::setAccelerationDimensionIndex(long Dimension){}


void CEpos2::saveParameters()
{
  this->writeObject(0x1010, 0x01, 0x65766173);

}

void CEpos2::restoreDefaultParameters()
{
  this->writeObject(0x1011, 0x01, 0x64616F6C);

}

long CEpos2::getRS232Baudrate()
{
  std::stringstream s;
  long ans = this->readObject(0x2002, 0x00);

	s << "RS232 Baudrate: " <<std::hex<< ans << " / " <<std::dec<< ans << std::endl;
  this->p(s);
	return(ans);
}

void CEpos2::setRS232Baudrate(long baudrate)
{
  this->writeObject(0x2002, 0x00, baudrate);
}

long CEpos2::getRS232FrameTimeout()
{
  std::stringstream s;
  long ans = this->readObject(0x2005, 0x00);

	s << "RS232 Frame Timeout: " <<std::hex<< ans << " / " <<std::dec<< ans << std::endl;
  this->p(s);
	return(ans);
}

void CEpos2::setRS232FrameTimeout(long timeout)
{
  this->writeObject(0x2005, 0x00, timeout);
}

long CEpos2::getUSBFrameTimeout()
{
  std::stringstream s;
  long ans = this->readObject(0x2006, 0x00);

	s << "USB Frame Timeout: " <<std::hex<< ans << " / " <<std::dec<< ans << std::endl;
  this->p(s);
	return(ans);
}

void CEpos2::setUSBFrameTimeout(long timeout)
{
  this->writeObject(0x2006, 0x00, timeout);
}

long CEpos2::getNegativeLong(long positiu)
{
  if(positiu>32767){
    return(positiu-65536);
  }else{
    return(positiu);
  }
}


// #############################   I/O   ######################################

long CEpos2::getAnalogOutput1(){return 1;}

void CEpos2::setAnalogOutput1(long voltage_mV){}

// #############################   MARKER POSITION   ##########################

long CEpos2::getPositionMarker(int buffer)
{
  int obj;
  switch(buffer)
  {
    case 0:
      obj = 1;
      break;
    case 1:
      obj = 5;
      break;
    case 2:
      obj = 6;
      break;
  }
  std::stringstream s;
  long ans = this->readObject(0x2074, obj);

  s << " Marker Position " <<std::hex<< ans << " / " <<std::dec<< ans << std::endl;
  this->p(s);
  return(ans);
}

void CEpos2::setPositionMarker(char mode, char polarity, char edge_type, char digitalIN)
{
  // set the digital input as position marker & options
  this->writeObject(0x2070, digitalIN, 4);
  // mask (which functionalities are active) (bit 3 0x0008)
  this->writeObject(0x2071, 0x02, 0x0008);
  // execution (if set the function executes) (bit 3 0x0008)
  this->writeObject(0x2071, 0x04, 0x0008);

  // options
  this->writeObject(0x2071, 0x03, polarity);
  this->writeObject(0x2074, 0x02, edge_type);
  this->writeObject(0x2074, 0x03, mode);

}

void CEpos2::startPositionMarker()
{
  this->threads = CThreadServer::instance();
  this->threads->create_thread(this->position_marked_thread_id);
  this->threads->attach_thread(this->position_marked_thread_id,
                               this->threadPositionMarker, this);
  this->threads->start_thread(this->position_marked_thread_id);
}

void CEpos2::stopPositionMarker()
{
  this->events->set_event(this->getStopPositionMarkerEventId());
}

void* CEpos2::threadPositionMarker(void *param)
{
  CEpos2 *self = (CEpos2*)param;
  long markpos=0, markposold=0;

  while( !self->events->event_is_set(self->stop_marking_event_id) )
  {
    markpos = self->getPositionMarker();
    if(markpos != markposold)
    {
      self->events->set_event(self->getPositionMarkerEventId());
      markposold = markpos;
    }
    // minimum freq = 0.05s = 20Hz
    usleep(1000000*0.05);
  }
  std::cout << "thread out\n";
  pthread_exit(NULL);

}


// #############################   HOMING   ###################################



void CEpos2::setHoming(int home_method, int speed_pos, int speed_zero,
                       int acc, int digitalIN)
{
  // set digital input as home switch
  this->writeObject(0x2070, digitalIN, 3);
  // mask
  this->writeObject(0x2071, 0x02, 0x0004);
  this->writeObject(0x2071, 0x04, 0x000C);
  // options
  this->writeObject(0x6098, 0x00, home_method);
  this->writeObject(0x6099, 0x01, speed_pos);
  this->writeObject(0x6099, 0x02, speed_zero);
  this->writeObject(0x609A, 0x00, acc);
}

void CEpos2::doHoming()
{
  this->writeObject(0x6040, 0x00, 0x001F);

  this->threads->start_thread(this->target_reached_thread_id);
}

void CEpos2::stopHoming()
{
  this->events->set_event(this->getTargetReachedEventId());
  this->writeObject(0x6040, 0x00, 0x010F);
  this->threads->end_thread(this->target_reached_thread_id);
  this->events->reset_event(this->getTargetReachedEventId());
}

void* CEpos2::threadTargetReached(void *param)
{
  CEpos2 *self = (CEpos2*)param;

  bool tr=false;

  while(!tr)
  {
    tr = self->isTargetReached();
    // minimum freq = 0.05s = 20Hz
    usleep(1000000*0.05);
  }
  self->events->set_event(self->getTargetReachedEventId());

  pthread_exit(NULL);
}

std::string CEpos2::getTargetReachedEventId()
{
  return(this->target_reached_event_id);
}

std::string CEpos2::getPositionMarkerEventId()
{
  return(this->position_marked_event_id);
}

std::string CEpos2::getStopPositionMarkerEventId()
{
  return(this->stop_marking_event_id);
}

// #############################   DIG IN   ###################################





int CEpos2::getDigInState(int digitalIN)
{
  std::stringstream s;
  int ans = this->readObject(0x2070, digitalIN);

  s << "Digital In " << digitalIN << ": " <<std::hex<< ans << " / " <<std::dec<< ans;
  this->p(s);
  return(ans);
}

int CEpos2::getDigInStateMask()
{
  std::stringstream s;
  int ans = this->readObject(0x2071, 0x01);

  s << "State Mas k: " <<std::hex<< ans << " / " <<std::dec<< ans;
  this->p(s);
  return(ans);
}

int CEpos2::getDigInFuncMask()
{
  std::stringstream s;
  int ans = this->readObject(0x2071, 0x02);

  s << "Functionalities Mask: " <<std::hex<< ans << " / " <<std::dec<< ans;
  this->p(s);
  return(ans);
}

int CEpos2::getDigInPolarity()
{
  std::stringstream s;
  int ans = this->readObject(0x2071, 0x03);

  s << "Polarity Mask: " <<std::hex<< ans << " / " <<std::dec<< ans ;
  this->p(s);
  return(ans);
}

int CEpos2::getDigInExecutionMask()
{
  std::stringstream s;
  int ans = this->readObject(0x2071, 0x04);

  s << "Execution Mask: " <<std::hex<< ans << " / " <<std::dec<< ans;
  this->p(s);
  return(ans);
}


// deprecated


void CEpos2::setHomePosition(long home_position_qc)
{
  this->writeObject(0x2081, 0x00,home_position_qc);
}
long CEpos2::getHomePosition()
{
  std::stringstream s;
  long ans = this->readObject(0x2081, 0x00);

  s << " Home Position " <<std::hex<< ans << " / " <<std::dec<< ans << std::endl;
  this->p(s);
  return(ans);
}

void CEpos2::setHome()
{
  char c;
  long home_pos=0;

  long mode_anterior = this->getOperationMode();
  this->disableOperation();
  std::cout << "    [EPOS2] Move Load to 0 position and press a key ";
  std::cin >> c;
  std::cout << std::endl;
  std::cout << "    [EPOS2] Wait until process finishes" << std::endl;
  this->enableOperation();
  home_pos = this->readPosition();
  this->setOperationMode(HOMING);
  this->getOperationMode();
  this->setHomePosition(home_pos);
  this->getHomePosition();
  this->setOperationMode(mode_anterior);
  this->getOperationMode();
  std::cout << "    [EPOS2] Restart EPOS2 (unplug from current) after that the new home will be set" << std::endl;
}

const std::string CEpos2::error_names[] = {
  "Generic Error",
  "Current Error",
  "Voltage Error",
  "Temperature Error",
  "Communication Error",
  "Device profile specific",
  "Motion Error"
};

const int CEpos2::error_codes[]={
  0x0000,
  0x1000,
  0x2310,
  0x3210,
  0x3220,
  0x4210,
  0x5113,
  0x5114,
  0x6100,
  0x6320,
  0x7320,
  0x8110,
  0x8111,
  0x8120,
  0x8130,
  0x8150,
  0x81FD,
  0x81FE,
  0x81FF,
  0x8210,
  0x8611,
  0xFF01,
  0xFF02,
  0xFF03,
  0xFF04,
  0xFF05,
  0xFF06,
  0xFF07,
  0xFF08,
  0xFF09,
  0xFF0A,
  0xFF0B,
  0xFF0C,
  0xFF0D
};

const std::string CEpos2::error_descriptions[]={
  "No Error",
  "Generic Error",
  "Over Current Error",
  "Over Voltage Error",
  "Under Voltage",
  "Over Temperature",
  "Supply Voltage (+5V) Too Low",
  "Supply Voltage Output Stage Too Low",
  "Internal Software Error",
  "Software Parameter Error",
  "Sensor Position Error",
  "CAN Overrun Error (Objects lost)",
  "CAN Overrun Error",
  "CAN Passive Mode Error",
  "CAN Life Guard Error",
  "CAN Transmit COB-ID collision",
  "CAN Bus Off",
  "CAN Rx Queue Overrun",
  "CAN Tx Queue Overrun",
  "CAN PDO length Error",
  "Following Error",
  "Hall Sensor Error",
  "Index Processing Error",
  "Encoder Resolution Error",
  "Hallsensor not found Error",
  "Negative Limit Error",
  "Positive Limit Error",
  "Hall Angle detection Error",
  "Software Position Limit Error",
  "Position Sensor Breach",
  "System Overloaded",
  "Interpolated Position Mode Error",
  "Autotuning Identification Error"
};















