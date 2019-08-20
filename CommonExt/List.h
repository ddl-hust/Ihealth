#pragma once

#ifndef COMMONEXT_LIST
#define COMMONEXT_LIST

#include "Collection.h"
#include "MemoryStrategy.h"
#include "Sort.h"
#include "ExtCPP.h" // C++ �﷨��չ

namespace Ext
{

	namespace Collection
	{
		/// <summary>
		/// ��ʾ�����б������ࡣ
		/// </summary>
		template <class T> 
		class __declspec(dllexport) List
		{
		public:

			/// <summary>
			/// ���ڱȽ����� T Ԫ�صĴ�С�ĺ���ָ�롣
			/// </summary>
			/// <remarks></remarks>
			/// <param name="a">[I] �Ƚϵĵ�һ�� T Ԫ�ص�ָ�롣</param>
			/// <param name="b">[I] �Ƚϵĵڶ��� T Ԫ�ص�ָ�롣</param>
			/// <returns>
			/// ����Ƚ�ʱ�ķ���ֵ��
			/// -1 if a < b is true
			///  1 if b < a is true
			///  0 if niether a < b nor b < a is true 
			/// 
			/// ����Ƚ�ʱ�ķ���ֵ��
			/// -1 if a > b is true
			///  1 if b > a is true
			///  0 if niether a < b nor b < a is true 
			/// </returns>
			/// <author name="hps" date="2012.6.8   9:42"/>
			typedef  int(*CompareFunc)(const T &a, const T &b);

		public:

			/// <summary>
			/// ��ʼ��һ���յ��б�������
			/// </summary>
			/// <remarks></remarks>
			/// <author name="hps" date="2012.6.5   11:59"/>
			List();

			/// <summary>
			/// ��ʼ��Ԥ������С���б�������
			/// </summary>
			/// <remarks></remarks>
			/// <param name=""></param>
			/// <author name="hps" date="2012.6.5   12:00"/>
			List(int capacity);

			/// <summary>
			/// �������캯����
			/// </summary>
			/// <remarks></remarks>
			/// <param name="rSource">[I] ָ��Ҫ������Դ�б�</param>
			/// <author name="hps" date="2012.6.5   12:02"/>
			List(const List<T>& rSource);

			/// <summary>
			/// ����������
			/// </summary>
			/// <remarks></remarks>
			/// <author name="hps" date="2012.6.5   12:03"/>
			virtual ~List();

			/// <summary>
			/// ��ֵ�������
			/// </summary>
			/// <remarks></remarks>
			/// <param name="&">[I] ָ�����ʽ����ֵ��</param>
			/// <returns>���ر��ʽ��ֵ�����á�</returns>
			/// <author name="hps" date="2012.6.5   12:03"/>
			List<T>& operator=(const List<T>& rSource);

			/// <summary>
			/// ���������б�
			/// </summary>
			/// <remarks>�����б���ڴ���Ч���޷�����ʱ�����ܵ��ô˷������˷������б��ڲ���������Ϣ����գ�����ʱ���ܵ�������ʱ������</remarks>
			/// <param name=""></param>
			/// <returns></returns>
			/// <author name="hps" date="2012.6.5   12:07"/>
			void EmergencyDestroy(void);

			/// <summary>
			/// ��ȡ�б���ʵ��Ԫ�ص���Ŀ
			/// </summary>
			/// <remarks></remarks>
			/// <returns>����ʵ�ʴ������б���Ԫ�ص���Ŀ��</returns>
			/// <author name="hps" date="2012.6.5   12:09"/>
			int Count() const;

			/// <summary>
			/// ��ȡ�б�������Ĵ�С��
			/// </summary>
			/// <remarks></remarks>
			/// <returns>�����б������Ĵ�С��</returns>
			/// <author name="hps" date="2012.6.5   12:10"/>
			int Capacity() const;

			/// <summary>
			/// ��ȡ�б�ʵ��ʹ���ڴ�Ĵ�С(���ֽ�Ϊ��λ)��
			/// </summary>
			/// <remarks></remarks>
			/// <returns>�����б�ʵ��ʹ���ڴ�Ĵ�С��</returns>
			/// <author name="hps" date="2012.6.5   12:14"/>
			unsigned int TotalMemory() const;

			/// <summary>
			/// �����������÷�����ȷ��������ֵ�� [0, capacity) ֮�䡣
			/// </summary>
			/// <remarks></remarks>
			/// <param name=""></param>
			/// <returns>������������Ԫ�ص�����</returns>
			/// <author name="hps" date="2012.6.5   12:12"/>
			T& operator[](int index);
			T& operator[](unsigned int index);
			const T& operator[](int index) const;
			const T& operator[](unsigned int index) const;

			/// <summary>
			/// ����ת������
			/// </summary>
			/// <remarks></remarks>
			/// <returns>�����б���ʼԪ�ص�ָ�롣</returns>
			/// <author name="hps" date="2012.6.5   12:15"/>
			operator T*();
			operator const T*() const;

			/// <summary>
			/// ��ȡ�б��еĵ�һ��Ԫ�ء�
			/// </summary>
			/// <remarks></remarks>
			/// <returns>���ص�һ��Ԫ�ص�ָ�롣���б�Ϊ�գ��򷵻� NULL��</returns>
			/// <author name="hps" date="2012.6.5   12:16"/>
			T& First();
			const T& First() const;

			/// <summary>
			/// ��ȡ�б��е����һ��Ԫ�ء�
			/// </summary>
			/// <remarks></remarks>
			/// <returns>�������һ��Ԫ�ص�ָ�롣���б�Ϊ�գ��򷵻� NULL��</returns>
			/// <author name="hps" date="2012.6.5   12:20"/>
			T& Last();
			const T& Last() const;

			/// <summary>
			/// ��ȡָ����������Ԫ�ء�
			/// </summary>
			/// <remarks>ע���÷��������������Χ����Ч�ԣ�������ķ�ΧԽ�磬�򷵻� NULL��</remarks>
			/// <param name="">[I] ָ��Ҫ������������</param>
			/// <returns>����ָ��������Ԫ�ص�ָ�롣</returns>
			/// <author name="hps" date="2012.6.5   12:17"/>
			T& At(int index);
			T& At(unsigned int index);
			const T& At(int index) const;
			const T& At(unsigned int index) const;

			/// <summary>
			/// ��ȡһ��ֵ��ֻ�ǵ�ǰ�� List �����Ƿ�Ϊ�ա�
			/// </summary>
			/// <remarks></remarks>
			/// <returns>�����������������κ�Ԫ�أ��򷵻� true�����򷵻� false��</returns>
			/// <author name="hps" date="2012.6.21   15:27"/>
			bool IsEmpty() const;

			/// <summary>
			/// ���б���׷��һ��Ԫ�����͵�һ������Ĭ��ֵ��Ԫ�ء�
			/// </summary>
			/// <remarks></remarks>
			/// <returns>������ӽ��б���Ԫ�ص����á�</returns>
			/// <author name="hps" date="2012.6.5   12:22"/>
			T& AppendNew();

			/// <summary>
			/// ��ָ��Ԫ�صĸ���׷�ӵ��б��ĩβ��
			/// </summary>
			/// <remarks></remarks>
			/// <param name="rElement">[I] ָ��Ҫ׷�ӵ�Ԫ�ء�</param>
			/// <returns></returns>
			/// <author name="hps" date="2012.6.5   12:26"/>
			void Append(const T& rElement);

			/// <summary>
			/// ��ָ�������Ԫ�صĸ���׷�ӵ��б��ĩβ��
			/// </summary>
			/// <remarks></remarks>
			/// <param name="count">[I] ָ����Ҫ������ pArray ��׷�ӵ��б��Ԫ�ص�������</param>
			/// <param name="pArray">[I] ָ�����顣</param>
			/// <returns></returns>
			/// <author name="hps" date="2012.6.5   12:50"/>
			void Append(int count, const T* pArray);

			/// <summary>
			/// ��ָ���б� list �е�����Ԫ��׷�ӵ���ǰ�б��С�
			/// </summary>
			/// <remarks></remarks>
			/// <param name="list">[I] ָ��Ҫ׷��Ԫ�ص��б�</param>
			/// <returns></returns>
			/// <author name="hps" date="2012.7.2   16:20"/>
			void Append(const List<T> &list);

			/// <summary>
			/// ���б�ĵ�ָ������������Ԫ�صĸ�����
			/// </summary>
			/// <remarks></remarks>
			/// <param name="index">[I] ָ����Ԫ�ز����λ�á�</param>
			/// <param name="rSource">[I] ָ��Ԫ�ء�</param>
			/// <returns></returns>
			/// <author name="hps" date="2012.6.5   13:19"/>
			void Insert(int index, const T& rSource);

			/// <summary>
			/// �Ƴ��б��е����һ��Ԫ�ء�
			/// </summary>
			/// <remarks></remarks>
			/// <returns></returns>
			/// <author name="hps" date="2012.6.5   13:21"/>
			void Remove();

			/// <summary>
			/// �Ƴ�ָ����������Ԫ�ء�
			/// </summary>
			/// <remarks></remarks>
			/// <param name="index">[I] ָ��Ҫ�Ƴ�Ԫ�ص�������</param>
			/// <returns></returns>
			/// <author name="hps" date="2012.6.5   13:22"/>
			void Remove(int index);

			/// <summary>
			/// ����б�
			/// </summary>
			/// <remarks></remarks>
			/// <returns></returns>
			/// <author name="hps" date="2012.6.5   13:23"/>
			void Clear();

			/// <summary>
			/// ��ת�б�
			/// </summary>
			/// <remarks></remarks>
			/// <returns></returns>
			/// <author name="hps" date="2012.6.5   13:23"/>
			void Reverse();

			/// <summary>
			/// ����ָ��������������Ԫ�ء�
			/// </summary>
			/// <remarks></remarks>
			/// <param name="index1">[I] ָ������1.</param>
			/// <param name="index2">[I] ָ������2.</param>
			/// <returns></returns>
			/// <author name="hps" date="2012.6.5   13:23"/>
			void Swap(int index1, int index2);

			/// <summary>
			/// ˳������б�
			/// </summary>
			/// <remarks></remarks>
			/// <param name="element">[I] ָ��Ҫ���ҵ�Ԫ�ء�</param>
			/// <param name="comparer">[I] ָ�����ڱȽϵ�����Ԫ�ش�С�ĺ���ָ�롣</param>
			/// <returns>������ָ��Ԫ����ȵĵ�һ������ֵ����û���ҵ���֮ƥ���Ԫ�أ��򷵻� -1.</returns>
			/// <author name="hps" date="2012.6.5   13:27"/>
			int Search(const T &key, CompareFunc pCompare) const;

			/// <summary>
			/// ʹ�ö��ֲ��ҷ�����������б��в���ָ����Ԫ�ء�
			/// </summary>
			/// <remarks>��ʹ�ø÷���֮ǰ������ȷ���б��е�Ԫ���Ѿ����򡣷��򷵻صĽ�����ܲ���ȷ��</remarks>
			/// <param name="element">[I] ָ��Ҫ���ҵ�Ԫ�ء�</param>
			/// <param name="pCompare">[I] ָ�����ڱȽϵ�����Ԫ�ش�С�ĺ���ָ�롣</param>
			/// <returns>������ָ��Ԫ����ȵĵ�һ������ֵ����û���ҵ���֮ƥ���Ԫ�أ��򷵻� -1.</returns>
			/// <author name="hps" date="2012.6.5   13:31"/>
			int BinarySearch(const T &key, CompareFunc pCompare) const;

			/// <summary>
			/// ʹ�ö������㷨�Ե�ǰ�б��������
			/// </summary>
			/// <remarks>�㷨����Ŀǰ���ã�����</remarks>
			/// <param name="pCompare">[I] ָ�����ڱȽϵ�����Ԫ�ش�С�ĺ���ָ�롣</param>
			/// <returns>������ɹ����򷵻� true�����򷵻� false��</returns>
			/// <author name="hps" date="2012.6.5   13:34"/>
			virtual bool HeapSort(CompareFunc pCompare);

			/// <summary>
			/// ʹ�ÿ��������㷨�Ե�ǰ�б��������
			/// </summary>
			/// <remarks></remarks>
			/// <param name="pCompare">[I] ָ�����ڱȽϵ�����Ԫ�ش�С�ĺ���ָ�롣</param>
			/// <returns>������ɹ����򷵻� true�����򷵻� false��</returns>
			/// <author name="hps" date="2012.6.5   13:36"/>
			virtual bool QuickSort(CompareFunc pCompare);

			/// <summary>
			/// Permutes the array so that output[i] = input[index[i]].
			/// The index[] array should be a permutation of (0,...,Count()-1).
			/// </summary>
			/// <remarks></remarks>
			/// <param name="inputArray"></param>
			/// <returns></returns>
			/// <author name="hps" date="2012.6.5   14:03"/>
			bool Permute(const int* inputArray);

			/// <summary>
			/// �����б��е�����Ԫ�ز�ʹ��Ĭ�Ϲ��캯���õ��Ķ�����������б�
			/// </summary>
			/// <remarks>�÷�������ı䵱ǰ�б��Ԫ��������������</remarks>
			/// <returns></returns>
			/// <author name="hps" date="2012.6.5   14:04"/>
			void Zero();

			/// <summary>
			/// Ԥ����ָ��������������
			/// </summary>
			/// <remarks></remarks>
			/// <param name="capacity"></param>
			/// <returns></returns>
			/// <author name="hps" date="2012.6.5   14:06"/>
			void Reserve(int capacity);

			/// <summary>
			/// ���δʹ�õ�������
			/// </summary>
			/// <remarks></remarks>
			/// <returns></returns>
			/// <author name="hps" date="2012.6.5   14:07"/>
			void Shrink();

			/// <summary>
			/// �����б�
			/// </summary>
			/// <remarks>�÷���ִ������б��Ԫ��������������������Ϊ 0��</remarks>
			/// <returns></returns>
			/// <author name="hps" date="2012.6.5   14:07"/>
			void Destroy();

			/// <summary>
			/// ΪԪ�ط����ڴ档
			/// </summary>
			/// <remarks>
			/// Ĭ������£�List ���ڲ�ʹ�� OnRealloc() ��ΪԪ�ط����ڴ档��Ҫʹ���µ��ڴ������ԣ�����д Realloc(...) ������
			/// ����д Realloc(...) ����ʱ��ȷ���÷�����������������
			/// 1). ��� ptr = NULL, �� capacity = 0���������� NULL��
			/// 2). ��� ptr = NULL���� capacity > 0, ������ capacity*sizeof(T) ��С���ڴ�鲢���ظ��ڴ���ָ�룻�������ڴ�ʧ�ܣ��򷵻� NULL��
			/// 3). ��� ptr != NULL, �� capacity = 0�����ͷ� ptr ִ�е��ڴ�鲢���� NULL��
			/// 4). ��� ptr != NULL, �� capacity > 0, �����·����ڴ�鲢�����ڴ���ָ�룻�������ڴ�ʧ�ܣ��򷵻� NULL��
			/// </remarks>
			/// <param name="ptr">[I] ָ��Ԫ�ص�ָ�롣</param>
			/// <param name="capacity">[I] ָ�������ڴ��������</param>
			/// <returns>����ָ�������ڴ��ָ�롣</returns>
			/// <author name="hps" date="2012.6.5   14:11"/>
			virtual T* Realloc(T* ptr, int capacity);

			/// <summary>
			/// �����б��ڲ�ʹ�õ�����ָ�롣
			/// </summary>
			/// <remarks></remarks>
			/// <returns></returns>
			/// <author name="hps" date="2012.6.5   14:23"/>
			T* Array();
			const T* Array() const;

			/// <summary>
			/// �����б���Ԫ�ص�������
			/// </summary>
			/// <remarks></remarks>
			/// <param name="count">[I] ָ��Ԫ�ص�������</param>
			/// <returns></returns>
			/// <author name="hps" date="2012.6.5   14:24"/>
			void SetCount(int count);

			/// <summary>
			/// �����б��������
			/// </summary>
			/// <remarks></remarks>
			/// <param name="capacity">[I] ָ���µ�������С��</param>
			/// <returns></returns>
			/// <author name="hps" date="2012.6.5   14:24"/>
			void SetCapacity(int capacity);

			/// <summary>
			/// ���б������������������ʱ��ʹ�ø÷��������µ�������
			/// </summary>
			/// <remarks></remarks>
			/// <returns></returns>
			/// <author name="hps" date="2012.6.5   14:25"/>
			int NewCapacity() const;

			/// <summary>
			/// ��ȡ�б�����������������
			/// </summary>
			/// <remarks></remarks>
			/// <returns>�����ڴ������Ĳ�����</returns>
			/// <author name="hps" date="2012.7.11   12:20"/>
			int GetMemoryStep() const;

			/// <summary>
			/// �����б�����������������
			/// </summary>
			/// <remarks></remarks>
			/// <param name="step">[I] ָ�������Ĳ�����</param>
			/// <returns></returns>
			/// <author name="hps" date="2012.7.11   12:21"/>
			void SetMemoryStep(int step);

			/// <summary>
			/// �����б��ڲ�ʹ�õ�����ָ�벢���ٵ�ǰ�б�
			/// </summary>
			/// <remarks>���÷��踺���ͷ�(���ã�OnFree(...))�÷������ص�ָ����ڴ档</remarks>
			/// <returns></returns>
			/// <author name="hps" date="2012.6.5   14:27"/>
			T* KeepArray();

			/// <summary>
			/// ����ʹ�ø÷�����
			/// </summary>
			/// <remarks></remarks>
			/// <param name="*"></param>
			/// <returns></returns>
			/// <author name="hps" date="2012.6.5   14:31"/>
			void SetArray(T*);

			/// <summary>
			/// ʹ��ָ����������Ϣ���õ�ǰ�б�
			/// </summary>
			/// <remarks></remarks>
			/// <param name="inputArray">[I] ָ�����顣</param>
			/// <param name="count">[I] ָ����С��</param>
			/// <param name="capacity">[I] ָ��������</param>
			/// <returns></returns>
			/// <author name="hps" date="2012.6.5   14:31"/>
			void SetArray(T* inputArray, int count, int capacity);

		public:
			Ext::CPP::Event OnAppending;
			Ext::CPP::Event OnAppended;
			Ext::CPP::Event OnRemoving;
			Ext::CPP::Event OnRemoved;

		protected:

			int BinarySearch(T a[], int low, int heigh, const T &key, CompareFunc pCompare) const;

			/// <summary>
			/// �ƶ�Ԫ��
			/// </summary>
			/// <remarks></remarks>
			/// <param name="destIndex">[I] Ŀ��������</param>
			/// <param name="srcIndex">[I] ԭ������</param>
			/// <param name="elementCount">[I] Ԫ��������</param>
			/// <returns></returns>
			/// <author name="hps" date="2012.6.5   14:34"/>
			void Move(int  destIndex, int  srcIndex, int  elementCount);

			/// <summary>
			/// ����Ĭ��Ԫ��ֵ��
			/// </summary>
			/// <remarks></remarks>
			/// <param name="t"></param>
			/// <returns></returns>
			/// <author name="hps" date="2012.6.5   14:35"/>
			void ConstructDefaultElement(T* t);

			/// <summary>
			/// ����Ԫ��
			/// </summary>
			/// <remarks></remarks>
			/// <param name="t"></param>
			/// <returns></returns>
			/// <author name="hps" date="2012.6.5   14:35"/>
			void DestroyElement(T& t);

			/// <summary>
			/// �ڲ����ڴ洢Ԫ�ص�����
			/// </summary>
			T*   m_a;        // pointer to array memory

			/// <summary>
			/// ��¼�б���ʵ��Ԫ�ص����������㣺0 <= m_count <= m_capacity
			/// </summary>
			int  m_count;

			/// <summary>
			/// ��¼ m_a �����ʵ�ʴ�С�����б��������
			/// </summary>
			int  m_capacity;

			/// <summary>
			/// �ڴ�����������Ĭ��Ϊ 8
			/// </summary>
			int  m_memoryStep;
		};

		// List ģ���ʵ��
		#include "ListImplement.h"

	} // End namespace Collection
} // End namespace Ext

#endif // COMMONEXT_LIST