/*
 * MochaException.h
 *
 *  Created on: Feb 6, 2012
 *      Author: nima
 */

#pragma once

#include <iostream>
#include <exception>
using namespace std;



class GuiException : public exception
{
private:
	const char *m_pWhat;
public:
    GuiException(const char *what);
    virtual ~GuiException()  throw() ;
	virtual const char* what();
};

