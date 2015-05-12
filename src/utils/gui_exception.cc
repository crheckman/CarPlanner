/*
 * MochaException.cpp
 *
 *  Created on: Feb 6, 2012
 *      Author: nima
 */

#include <CarPlanner/utils/gui_exception.h>



GuiException::GuiException(const char *what): m_pWhat(what) {
}

GuiException::~GuiException() throw()
{

}

const char* GuiException::what()
{
	return m_pWhat;
}


