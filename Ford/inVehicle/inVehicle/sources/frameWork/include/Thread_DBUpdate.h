/*******************************************************************************
*                           Continental Confidential
*                  Copyright (c) Continental, LLC. 2015
*
*      This software is furnished under license and may be used or
*      copied only in accordance with the terms of such license.
*******************************************************************************
* @file  Thread_DBUpdate.h
* @brief use TCP socket client to receive the GPS data from the server side.
*             parser the received message.
*             call Digital Horizon Data Update Manager to update data to Digital Horizon Database
*
* Change Log:
*      Date                Who             What
*      2014/12/25         Xin Shao        Create
*******************************************************************************
*/
#pragma once
unsigned int __stdcall Thread_DBUpdate(void *data);