#ifndef _PROPERTY_H_
#define _PROPERTY_H_

#include "HandleBase.h"
#include <list>
//#pragma warning(disable:4355)

namespace Ext
{

	namespace CPP
	{

		template<typename Arg>
		class GetHandle : public HandleBase<int, Arg>
		{
		public:
			static const GetHandle<Arg> Null;

		public:
			GetHandle() : HandleBase<int, Arg>()
			{
			}

			// �������캯����STL ������Ҫ�ú���
			GetHandle(const GetHandle &fun)
			{
				m_pFun = NULL;
				*this = fun;
			}

			// ȫ�ֺ�����̬����
			template <typename Fun>
			GetHandle(const Fun &fun) : HandleBase<int, Arg>(fun)
			{
			}

			// ��Ա����
			template <typename PointerToObj, typename PointerToMemFun>
			GetHandle(const PointerToObj &pObj, const PointerToMemFun &pFun) : HandleBase<int, Arg>(pObj, pFun)
			{
			}

			virtual ~GetHandle()
			{
			}

			GetHandle& operator= (const GetHandle &fun)
			{
				HandleBase<int, Arg>::operator= (fun);
				return *this;
			}

			bool operator== (const GetHandle &handler)
			{
				return HandleBase<int, Arg>::operator ==(handler);
			}
		};

		template<typename Arg>
		const GetHandle<Arg> GetHandle<Arg>::Null;

		template<typename Arg>
		class SetHandle : public HandleBase<int, Arg>
		{
		public:
			static const SetHandle<Arg> Null;

		public:

			SetHandle() : HandleBase<int, Arg>()
			{
			}

			// �������캯����STL ������Ҫ�ú���
			SetHandle(const SetHandle &fun)
			{
				m_pFun = NULL;
				*this = fun;
			}

			// ȫ�ֺ�����̬����
			template <typename Fun>
			SetHandle(const Fun &fun) : HandleBase<int, Arg>(fun)
			{
			}

			// ��Ա����
			template <typename PointerToObj, typename PointerToMemFun>
			SetHandle(const PointerToObj &pObj, const PointerToMemFun &pFun) : HandleBase<int, Arg>(pObj, pFun)
			{
			}

			virtual ~SetHandle()
			{
			}

			SetHandle& operator= (const SetHandle &fun)
			{
				HandleBase<int, Arg>::operator= (fun);
				return *this;
			}

			bool operator== (const SetHandle &handler)
			{
				return HandleBase<int, Arg>::operator ==(handler);
			}
		};

		template<typename Arg>
		const SetHandle<Arg> SetHandle<Arg>::Null;

		/// <summary>
		/// ��ͨ�Ķ�д����ģ��
		/// </summary>
		/// <remarks></remarks>
		/// <param name="Validate">[I] ָ��������֤���Իص��������������͡�</param>
		/// <param name="T">[I] ָ������ֵ�����͡�</param>
		template <typename T>
		class DciProperty
		{
		public:

			/// <summary>
			/// ����֤������
			/// </summary>
			/// <remarks></remarks>
			/// <param name="onStaticGet">[I] ָ�������ڻ�ȡֵʱ����֤������</param>
			/// <param name="onStaticSet">[I] ָ������������ֵʱ����֤������</param>
			/// <returns></returns>
			/// <author name="peishengh" date="2011.12.13   15:08"/>
			void Binding(const GetHandle<T> &onGet)
			{
				if (onGet.IsValid() == false)
					return;

				if (onGet.IsInstanceHandle())
				{
					this->m_OnGet = onGet;
				}
				else
				{
					this->m_OnStaticGet = onGet;
				}
			}

			void Binding(const SetHandle<T> &onSet)
			{
				if (onSet.IsValid() == false)
					return;

				if (onSet.IsInstanceHandle())
				{
					this->m_OnSet = onSet;
				}
				else
				{
					this->m_OnStaticSet = onSet;
				}
			}

			/// <summary>
			/// ����֤������
			/// </summary>
			/// <remarks></remarks>
			/// <param name="ownerInstance">[I] ʵ����֤�����Ķ���</param>
			/// <param name="onGet">[I] ָ�������ڻ�ȡֵʱ����֤������</param>
			/// <param name="onSet">[I] ָ������������ֵʱ����֤������</param>
			/// <returns></returns>
			/// <author name="peishengh" date="2011.12.13   15:07"/>
			void Binding(const GetHandle<T> &onGet, const SetHandle<T> &onSet)
			{
				this->Binding(onGet);
				this->Binding(onSet);
			}

			DciProperty& operator+= (const GetHandle<T> &handler)
			{
				this->Binding(handler);
				return *this;
			}

			DciProperty& operator+= (const SetHandle<T> &handler)
			{
				this->Binding(handler);
				return *this;
			}

			/// <summary>
			/// Ĭ�Ϲ��캯����
			/// </summary>
			/// <remarks></remarks>
			/// <author name="peishengh" date="2011.11.14   14:40"/>
			DciProperty()
			{
				this->Reset();
			}

			/// <summary>
			/// ʹ��Ĭ��ֵ��ʼ�����ԡ�
			/// </summary>
			/// <remarks></remarks>
			/// <param name="defaultValue">[I] ָ��Ĭ��ֵ��</param>
			/// <returns></returns>
			/// <author name="peishengh" date="2011.12.13   15:21"/>
			DciProperty(T defaultValue)
			{
				this->Reset();
				this->m_Value = defaultValue;
			}

			DciProperty(T defaultValue, const GetHandle<T> &onGet)
			{
				this->Reset();
				this->Binding(onGet);

				this->m_Value = defaultValue;
			}

			DciProperty(T defaultValue, const SetHandle<T> &onSet)
			{
				this->Reset();
				this->Binding(onSet);

				m_Value = defaultValue;
			}

			/// <summary>
			/// ʹ��Ĭ��ֵ�;�̬�ص���Ϣ��ʼ�����ԡ�
			/// </summary>
			/// <remarks></remarks>
			/// <param name="defaultValue">[I] ָ��Ĭ��ֵ��</param>
			/// <param name="onStaticGet">[I] ָ�������ڻ�ȡֵʱ����֤������</param>
			/// <param name="onStaticSet">[I] ָ������������ֵʱ����֤������</param>
			/// <returns></returns>
			/// <author name="peishengh" date="2011.12.13   15:23"/>
			DciProperty(T defaultValue, const GetHandle<T> &onGet, const SetHandle<T> &onSet)
			{
				this->Reset();
				this->Binding(onGet);
				this->Binding(onSet);

				m_Value = defaultValue;
			}

			/// <summary>
			/// �������캯����
			/// </summary>
			/// <remarks></remarks>
			/// <param name="rSource">[I] ָ��������Դ����</param>
			/// <author name="peishengh" date="2011.11.14   14:44"/>
			DciProperty(const DciProperty &rSource)
			{
				*this = rSource;
			}

			/// <summary>
			/// ��ֵ�������
			/// </summary>
			/// <remarks></remarks>
			/// <param name="rSource">[I] ָ����ֵ��Դ����</param>
			/// <returns>��ǰ����</returns>
			/// <author name="peishengh" date="2011.11.14   14:45"/>
			DciProperty& operator = (const DciProperty &rSource)
			{
				if (this != &rSource)
				{
					this->m_Value = rSource.m_Value;
				}
				return *this;
			}

			~DciProperty()
			{
				this->m_OnGet.Clear();
				this->m_OnSet.Clear();

				this->m_OnStaticGet.Clear();
				this->m_OnStaticSet.Clear();

				this->Reset();
			}

			/// <summary>
			/// ��д������ת����������
			/// </summary>
			/// <remarks>ʹ�õ�ǰ�����ֵ����ת��Ϊ����ֵ���͵�ֵ��</remarks>
			/// <returns>��������ֵ���͵�ֵ��</returns>
			/// <author name="peishengh" date="2011.11.14   14:46"/>
			operator T(void)
			{
				return this->GetValue(m_Value);
			}

			/// <summary>
			/// ��д������ת����������
			/// </summary>
			/// <remarks>ʹ�õ�ǰ�����ֵ����ת��Ϊ����ֵ���͵�ֵ��</remarks>
			/// <param name="">��������ֵ���͵�ֵ��</param>
			/// <returns></returns>
			/// <author name="peishengh" date="2011.12.13   15:26"/>
			operator T(void) const
			{
				T temp = m_Value;
				return this->GetValue(temp);
			}

			/// <summary>
			/// ��ʾ�ؽ���ǰ����ת��Ϊ����ֵ���Ͷ�Ӧ��ֵ��
			/// </summary>
			/// <remarks></remarks>
			/// <returns>��������ֵ���͵�ֵ��</returns>
			/// <author name="peishengh" date="2011.11.14   14:47"/>
			T& Raw(void)
			{
				return (this->GetValue(m_Value));
			}

			T& Raw(void) const
			{
				T temp = m_Value;
				return (this->GetValue(temp));
			}

			/// <summary>
			/// ��ֵ�������
			/// </summary>
			/// <remarks>ʹ������ֵ���͵�ֵ�����໥ת��Ϊ��ǰ�����ֵ��</remarks>
			/// <param name="value">[I] ָ������ֵ��</param>
			/// <returns>��������ֵ���͵�ֵ��</returns>
			/// <author name="peishengh" date="2011.11.14   14:48"/>
			T& operator = (T value)
			{
				if (m_IsOnSetting == true)
					return m_Value;

				m_IsOnSetting = true;

				// ��ʼ�������Ե�ֵ
				//if (m_OnStaticSet.size() != 0)
				if (m_OnStaticSet.IsValid())
				{
					int result = m_OnStaticSet.Invoke(value);
					if (result == 0)
						m_Value = value;
				}

				//if (m_OnSet.size() != 0)
				if (m_OnSet.IsValid())
				{
					int result = m_OnSet.Invoke(value);
					if (result == 0)
						m_Value = value;
				}

				if (m_OnStaticSet.IsValid() == false && m_OnSet.IsValid() == false)
					m_Value = value;

				// �������Ե�ֵ����
				m_IsOnSetting = false;

				return m_Value;
			}

		private:

			void Reset()
			{
				m_OnGet.Clear();
				m_OnSet.Clear();
				m_OnStaticGet.Clear();
				m_OnStaticSet.Clear();

				m_IsOnSetting = false;
			}

			/// <summary>
			/// ��ȡ��ǰ������ֵ��
			/// </summary>
			/// <remarks></remarks>
			/// <returns>���ص�ǰ������ֵ��</returns>
			/// <author name="peishengh" date="2011.11.14   14:51"/>
			T& GetValue(T& v) const
			{		
				if (this->m_OnStaticGet.IsValid() == true)
					m_OnStaticGet.Invoke(v);

				if (this->m_OnGet.IsValid() == true)
					m_OnGet.Invoke(v);

				return v;
			}

		protected:
			/// <summary>
			/// �������Ե�ֵ��
			/// </summary>
			T m_Value;

			/// <summary>
			/// ʵ������ָ�룺�ڻ�ȡֵʱ������֤��
			/// </summary>
			GetHandle<T> m_OnGet;

			/// <summary>
			/// ʵ������ָ�룺������ֵʱ������֤��
			/// </summary>
			SetHandle<T> m_OnSet;

			/// <summary>
			/// ��̬����ָ�룺�ڻ�ȡֵʱ������֤��
			/// </summary>
			GetHandle<T> m_OnStaticGet;

			/// <summary>
			/// ��̬����ָ�룺������ֵʱ������֤��
			/// </summary>
			SetHandle<T> m_OnStaticSet;

			/// <summary>
			/// ���������Ե�ֵ�ṩ״̬���������ݹ��Ƕ����������ֵ��
			/// </summary>
			bool m_IsOnSetting;
		};

		/// <summary>
		/// ��ͨ�Ķ�д����ģ��
		/// </summary>
		/// <remarks></remarks>
		/// <param name="Validate">[I] ָ��������֤���Իص��������������͡�</param>
		/// <param name="T">[I] ָ������ֵ�����͡�</param>
		template <typename T>
		class ReadOnly
		{
		public:

			/// <summary>
			/// ����֤������
			/// </summary>
			/// <remarks></remarks>
			/// <param name="onStaticGet">[I] ָ�������ڻ�ȡֵʱ����֤������</param>
			/// <param name="onStaticSet">[I] ָ������������ֵʱ����֤������</param>
			/// <returns></returns>
			/// <author name="peishengh" date="2011.12.13   15:08"/>
			void Binding(const GetHandle<T> &onGet)
			{
				if (onGet.IsValid() == false)
					return;

				if (onGet.IsInstanceHandle())
				{
					this->m_OnGet = onGet;
				}
				else
				{
					this->m_OnStaticGet = onGet;
				}
			}

			ReadOnly& operator+= (const GetHandle<T> &handler)
			{
				this->Binding(handler);
				return *this;
			}

			/// <summary>
			/// Ĭ�Ϲ��캯����
			/// </summary>
			/// <remarks></remarks>
			/// <author name="peishengh" date="2011.11.14   14:40"/>
			ReadOnly()
			{
				this->Reset();
			}

			ReadOnly(T defaultValue, const GetHandle<T> &onGet)
			{
				this->Reset();
				this->Binding(onGet);

				this->m_Value = defaultValue;
			}

			/// <summary>
			/// �������캯����
			/// </summary>
			/// <remarks></remarks>
			/// <param name="rSource">[I] ָ��������Դ����</param>
			/// <author name="peishengh" date="2011.11.14   14:44"/>
			ReadOnly(const ReadOnly &rSource)
			{
				*this = rSource;
			}

			/// <summary>
			/// ��ֵ�������
			/// </summary>
			/// <remarks></remarks>
			/// <param name="rSource">[I] ָ����ֵ��Դ����</param>
			/// <returns>��ǰ����</returns>
			/// <author name="peishengh" date="2011.11.14   14:45"/>
			ReadOnly& operator = (const ReadOnly &rSource)
			{
				if (this != &rSource)
				{
					this->m_Value = rSource.m_Value;
				}
				return *this;
			}

			~ReadOnly()
			{
				this->m_OnGet.Clear();
				this->m_OnStaticGet.Clear();

				this->Reset();
			}

			/// <summary>
			/// ��д������ת����������
			/// </summary>
			/// <remarks>ʹ�õ�ǰ�����ֵ����ת��Ϊ����ֵ���͵�ֵ��</remarks>
			/// <returns>��������ֵ���͵�ֵ��</returns>
			/// <author name="peishengh" date="2011.11.14   14:46"/>
			operator T(void)
			{
				return this->GetValue(m_Value);
			}

			/// <summary>
			/// ��д������ת����������
			/// </summary>
			/// <remarks>ʹ�õ�ǰ�����ֵ����ת��Ϊ����ֵ���͵�ֵ��</remarks>
			/// <param name="">��������ֵ���͵�ֵ��</param>
			/// <returns></returns>
			/// <author name="peishengh" date="2011.12.13   15:26"/>
			operator T(void) const
			{
				T temp = m_Value;
				return this->GetValue(temp);
			}

			/// <summary>
			/// ��ʾ�ؽ���ǰ����ת��Ϊ����ֵ���Ͷ�Ӧ��ֵ��
			/// </summary>
			/// <remarks></remarks>
			/// <returns>��������ֵ���͵�ֵ��</returns>
			/// <author name="peishengh" date="2011.11.14   14:47"/>
			T To(void)
			{
				return this->GetValue(m_Value);
			}

			T To(void) const
			{
				T temp = m_Value;
				return this->GetValue(temp);
			}

		private:

			void Reset()
			{
				m_OnGet.Clear();
				m_OnStaticGet.Clear();
			}

			/// <summary>
			/// ��ȡ��ǰ������ֵ��
			/// </summary>
			/// <remarks></remarks>
			/// <returns>���ص�ǰ������ֵ��</returns>
			/// <author name="peishengh" date="2011.11.14   14:51"/>
			T GetValue(T& v) const
			{
				if (this->m_OnStaticGet.IsValid() ==  true)
					m_OnStaticGet.Invoke(v);

				if (this->m_OnGet.IsValid() == true)
					m_OnGet.Invoke(v);

				return v;
			}

		private:
			/// <summary>
			/// �������Ե�ֵ��
			/// </summary>
			T m_Value;

			/// <summary>
			/// ʵ������ָ�룺�ڻ�ȡֵʱ������֤��
			/// </summary>
			GetHandle<T> m_OnGet;

			/// <summary>
			/// ��̬����ָ�룺�ڻ�ȡֵʱ������֤��
			/// </summary>
			GetHandle<T> m_OnStaticGet;
		};

	}// End namespace CPP

} // End namespace Ext
#endif // _PROPERTY_H_