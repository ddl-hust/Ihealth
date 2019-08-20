#pragma once

#ifdef DCICOMMONEXT_MODULE
#define DCICOMMONEXT_MODULE_EXPIMP __declspec(dllexport)
#else
#define DCICOMMONEXT_MODULE_EXPIMP __declspec(dllimport)
#endif

#include "List.h"
using namespace Ext::Collection;

#include <vector>

namespace Ext
{

	/// <summary>
	/// �ṩһ�龲̬������Э�����һЩͨ�õ��߼�������
	/// </summary>
	class DCICOMMONEXT_MODULE_EXPIMP CTool
	{
	public:
		CTool(void);
		~CTool(void);

		static int CompareStringWithNoCase(const CString &a, const CString &b);

		/// <summary>
		/// �ͷ� std::vector<T*> ��������Դ��
		/// </summary>
		/// <remarks></remarks>
		/// <param name="vect">[I] ָ��Ҫ�ͷ���Դ��������</param>
		/// <returns></returns>
		/// <author name="hps" date="2012.5.22   9:27"/>
		template<typename T> static void ReleaseVector(std::vector<T*> &vect)
		{
			for (std::vector<T*>::const_iterator begin = vect.begin(); begin != vect.end(); ++begin)
			{
				delete (*begin);
			}
			vect.clear();
		}

		template<typename T> static void ReleaseList(List<T*> &list)
		{
			int count = list.Count();
			for (int i = 0; i < count; ++i)
			{
				T *p = list[i];
				if (p == NULL)
					continue;

				delete p;
			}
			list.Clear();
			list.Shrink();
		}
	};

}