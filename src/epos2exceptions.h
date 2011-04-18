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

#ifndef EPOS2_EXCEPTIONS
#define EPOS2_EXCEPTIONS

#include "exceptions.h"

/**
 * \brief EPOS2 exception class
 *
 * This class implements the exceptions for the CEpos2 class.
 *
 * Also, similarly to other exception classes, it appends a class identifer
 * string ("[CEpos2 class] - ") to the error message in order to identify the
 * class that generated the exception.
 *
 * The base class can be used to catch any exception thrown by the application
 * or also, this class can be used in order to catch only exceptions generated 
 * by CEpos2 objects.
 */
class CEpos2Exception : public CException
{
	public:
	/**
	 * \brief Constructor
	 *
	 * The constructor calls the base class constructor to add the general
	 * exception identifier and then adds the class identifier string 
	 * "[CComm class]" and the supplied error message. 
	 *
	 * The total exception message will 
     	 * look like this:
	 *
	 * \verbatim  [Exception caught] - [CEpos2 class] - <error message> \endverbatim
	 *
   * \param where a null terminated string that contains where has been the 
                  exception. Generally "_HERE_"
	 * \param error_msg a null terminated string that contains the error message.
	 *                  This string may have any valid character and there is no
	 *                  limit on its length.
	 *
	 */
    CEpos2Exception(const std::string& where, const std::string& error_msg);
};

#endif
