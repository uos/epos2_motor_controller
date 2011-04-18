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

#include <stdio.h>
#include <unistd.h>

#include <iostream>
#include "ctime.h"
#include "Epos2.h"

using namespace std;

/**
 * \example test_delays.cpp
 *
 * This is a test to calculate average delays in epos2
 *
 */

int main(void){

	CEpos2 controller;

	cout << "Epos2 Delay:" << endl;

  long pos;
	CTime tavg;

	try{

    controller.init();

    // enable motor in velocity mode
    controller.enableController();
    controller.enableMotor(controller.VELOCITY);
    controller.setOperationMode(controller.VELOCITY);

    controller.setTargetVelocity(100);

    // start the movement
    controller.startVelocity();
    CTime tbase,telapsed,t0,t1;
    double avg=0,total_avg=0,max_avg=0,min_avg=0;

    do{

      t0.set();
      pos = controller.readPosition();
      t1.set();
      avg = (t1-t0).getTimeInMilliseconds();
      if(total_avg==0)
        total_avg = avg;
      else
        total_avg = (total_avg + avg)/2;

      max_avg = max(max_avg,avg);
      min_avg = min(min_avg,avg);

      telapsed.set();
      telapsed = telapsed - tbase;

    }while( telapsed.getTimeInSeconds() < 10 );


    controller.stopVelocity();

    cout <<
        " avg: " << total_avg <<
        " min: " << min_avg <<
        " max: " << max_avg << endl;

    controller.close();

	}catch(CException &e){
    controller.close();
		cout << e.what() << endl;
	}

	return 1;

}

