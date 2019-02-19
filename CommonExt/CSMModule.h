/********************************************************************
	Created:	2013.10.11  14:03
	Author:		hps
	
	Purpose:	��Ϊ CSM ��ģ��ʱ��Ҫ����һЩ����ĳ�ʼ��
	Remark:		
*********************************************************************/

#pragma once

#ifdef DCICOMMONEXT_MODULE
#define DCICOMMONEXT_MODULE_EXPIMP __declspec(dllexport)
#else
#define DCICOMMONEXT_MODULE_EXPIMP __declspec(dllimport)
#endif

#include "RPC.h"
#include "RPCClient.h"
#include "RPCServer.h"
#include "RPCResponse.h"
#include "ExtCPP.h"
using namespace Ext::CPP;

namespace Ext
{
	/// <summary>
	/// ��ʾ CSM �ܹ��µ�һ����ģ����ӷ���
	/// </summary>
	class DCICOMMONEXT_MODULE_EXPIMP CSMModule
	{
	private:
		static Panic getRootServiceAddr(char *ip, u_short *port);
		static void initDefaultRPC();

	public:
		CString Name;
		RPCClient *Client;
		RPCServer *Server;

		static CSMModule *DefaultModule;

		CSMModule();
		CSMModule(RPCClient *client, RPCServer *server);
		~CSMModule();

		// һ�������ʹ�� DefaultModule ���ü���
		static Panic SetName(const CString &moduleName);
		static Panic MakesureCSMReady();
		static Panic SetupRPC();
		static Panic TeardownRPC();
		static Panic Release();

		// ʵ�巽�����ã�һ������ͬʱ��Ϊ CSM �Ķ����ģ�������ʹ�á�Ins means Instance��
		Panic InsSetName(const CString &moduleName);
		Panic InsMakesureCSMReady();
		Panic InsSetupRPC();
		Panic InsTeardownRPC();
		Panic InsRelease();
	};
}