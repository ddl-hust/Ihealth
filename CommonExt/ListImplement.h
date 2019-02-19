
#ifndef COMMONEXT_LISTIMPLEMENT
#define COMMONEXT_LISTIMPLEMENT

#if defined(ON_COMPILER_MSC)


// ��ʹ�� /w ����ʱ�������˲���Ҫ�ľ���
#pragma warning(push)

// List<T>::DestroyElement ģ�庯��������һ��δʹ�õĲ����ľ���
#pragma warning(disable:4100)

// ComareAsc<T> �� ComapreDes<T> ��������ʵ����ͬʱʹ���� static ���Σ������˲���Ҫ�ľ���
#pragma warning(disable:4211)
#endif

template <class T>
List<T>::List()
: m_a(0),
m_count(0),
m_capacity(0),
m_memoryStep(8)
{
}

template <class T>
List<T>::List(int capacity)
: m_a(0),
m_count(0),
m_capacity(0),
m_memoryStep(8)
{
	if (capacity > 0) 
		SetCapacity(capacity);
}

template <class T>
List<T>::List(const List<T>& rSource)
: m_a(0),
m_count(0),
m_capacity(0),
m_memoryStep(8)
{
	*this = rSource;
}

template <class T>
List<T>::~List()
{
	SetCapacity(0);
}

template <class T>
List<T>& List<T>::operator=(const List<T>& rSource)
{
	if (&rSource != this)
	{
		if (rSource.m_count <= 0) 
		{
			m_count = 0;
		}
		else 
		{
			if (m_capacity < rSource.m_count) 
			{
				SetCapacity(rSource.m_count);
			}
			if (m_a != NULL) 
			{
				m_count = rSource.m_count;
				for (int i = 0; i < m_count; i++) 
				{
					m_a[i] = rSource.m_a[i];
				}
			}
		}

		OnAppending = rSource.OnAppending;
		OnAppended  = rSource.OnAppended;
		OnRemoving  = rSource.OnRemoving;
		OnRemoved   = rSource.OnRemoved;
	}
	return *this;
}

template <class T>
int List<T>::NewCapacity() const
{
#if 1
	return (m_count + m_memoryStep);
#else
	// cap_size = 128 MB on 32-bit os, 256 MB on 64 bit os
	// ���б���Ԫ����ʹ�õ��ڴ�ռ��� cap_size ֮�ڣ��������Է�������ʽ����
	const size_t cap_size = 32 * sizeof(void*) * 1024 * 1024;
	if (m_count*sizeof(T) <= cap_size || m_count < 8)
		return ((m_count <= 2) ? 4 : 2*m_count);

	// ���б���Ԫ����ʹ�õ��ڴ�ռ��ѳ��� cap_size �Ĵ�С���������Թ̶���������ʽ����
	int delta_count = 8 + cap_size/sizeof(T);
	if (delta_count > m_count)
		delta_count = m_count;

	return (m_count + delta_count);
#endif
}

template <class T>
int List<T>::GetMemoryStep() const
{
	return m_memoryStep;
}

template <class T>
void List<T>::SetMemoryStep(int step)
{
	if (step > 0)
		m_memoryStep = step;
}


template <class T>
T* List<T>::Realloc(T* ptr, int capacity)
{
	return (T*)CMemoryStrategy::OnRealloc(ptr, capacity * sizeof(T));
}

template <class T>
void List<T>::EmergencyDestroy(void)
{
	m_count = 0;
	m_capacity = 0;
	m_a = 0;
}

template <class T>
int List<T>::Count() const
{
	return m_count;
}

template <class T>
int List<T>::Capacity() const
{
	return m_capacity;
}

template <class T>
unsigned int List<T>::TotalMemory() const
{
	return ((unsigned int)(m_capacity * sizeof(T)));
}

template <class T>
T& List<T>::operator[](int i)
{ 
	return m_a[i]; 
}

template <class T>
T& List<T>::operator[](unsigned int i)
{ 
	return m_a[i]; 
}

template <class T>
const T& List<T>::operator[](int i) const
{
	return m_a[i];
}

template <class T>
const T& List<T>::operator[](unsigned int i) const
{
	return m_a[i];
}

template <class T>
List<T>::operator T*()
{
	return (m_count > 0) ? m_a : 0;
}

template <class T>
List<T>::operator const T*() const
{
	return (m_count > 0) ? m_a : 0;
}

template <class T>
T* List<T>::Array()
{
	return m_a;
}

template <class T>
const T* List<T>::Array() const
{
	return m_a;
}

template <class T>
T* List<T>::KeepArray()
{
	T* p = m_a;
	m_a = 0;
	m_count = 0;
	m_capacity = 0;
	return p;
}

template <class T>
void List<T>::SetArray(T* p)
{
	if (m_a != NULL && m_a != p)
		Destroy();
	m_a = p;
}

template <class T>
void List<T>::SetArray(T* p, int count, int capacity)
{
	if (m_a != NULL && m_a != p)
		Destroy();

	m_a = p;
	m_count = count;
	m_capacity = capacity;
}

template <class T>
T& List<T>::First()
{ 
	return m_a[0]; 
	//return (m_count > 0) ? m_a : 0;
}

template <class T>
const T& List<T>::First() const
{
	return m_a[0]; 
	//return (m_count > 0) ? m_a : 0;
}

template <class T>
T& List<T>::At(int i)
{ 
	return m_a[i]; 
}

template <class T>
T& List<T>::At(unsigned int i)
{ 
	return m_a[i];
}

template <class T>
const T& List<T>::At(int i) const
{
	return m_a[i];
}

template <class T>
const T& List<T>::At(unsigned int i) const
{
	return m_a[i];
}

template <class T>
T& List<T>::Last()
{ 
	return m_a[m_count-1];
	//return (m_count > 0) ? m_a+(m_count-1) : 0;
}

template <class T>
const T& List<T>::Last() const
{
	return m_a[m_count-1];
	//return (m_count > 0) ? m_a+(m_count-1) : 0;
}

template <class T>
bool List<T>::IsEmpty() const
{
	return (m_count == 0) ? true : false;
}

template <class T>
void List<T>::Move(int dest_i, int src_i, int ele_cnt)
{
	// �����б��в���Ԫ�ػ��Ƴ�Ԫ��ʱ����Ҫʱ����Ҫ�ƶ��Ѵ��ڵ�Ԫ��
	if (ele_cnt <= 0 || src_i < 0 || dest_i < 0 || src_i == dest_i || 
		src_i + ele_cnt > m_count || dest_i > m_count)
		return;

	int capacity = dest_i + ele_cnt;
	if (capacity > m_capacity ) 
	{
		if (capacity < 2*m_capacity)
			capacity = 2*m_capacity;

		SetCapacity(capacity);
	}

	memmove(&m_a[dest_i], &m_a[src_i], ele_cnt*sizeof(T));
}

template <class T>
void List<T>::ConstructDefaultElement(T* p)
{
	// ���� new operator �����������£�
	//	1������ placement new�����ﲻ�����ڴ棬ֻ���ش����ָ���ڴ��ָ�롣
	//	2�����ù��캯����ʼ������
	new(p) T;
}

template <class T>
void List<T>::DestroyElement(T& x)
{
	//  ע�⣺��ʹ�� placement new ����Ķ��󣬲���ʹ�� delete ���ͷŶ������Դ��Ӧͨ�����ö���������������ͷ���Դ
	x.~T();
}

template <class T>
T& List<T>::AppendNew()
{
	if (m_count == m_capacity) 
	{
		int newcapacity = NewCapacity();
		Reserve( newcapacity );
	}
	else
	{
		// First destroy what's there ..
		DestroyElement(m_a[m_count]);

		// and then get a properly initialized element
		ConstructDefaultElement(&m_a[m_count]);
	}
	return m_a[m_count++];
}

template <class T>
void List<T>::Append(const T& x) 
{
	if (m_count == m_capacity) 
	{
		const int newcapacity = NewCapacity();
		if (m_a != NULL)
		{
			// �б��е�Ԫ���ٴ����
			const int s = (int)(&x - m_a);
			if (s >= 0 && s < m_capacity)
			{
				// List �����е�Ԫ�ز���Ҫʵ�ֿ������캯����������ʵ�ָ�ֵ�����
				T temp;
				temp = x;

				Reserve( newcapacity );

				// ��������׷��Ԫ���¼�
				EventArg arg(this, &temp);
				this->OnAppending.Raise(&arg);

				// ������Ԫ��
				m_a[m_count++] = temp;

				// ����׷��Ԫ������¼�
				this->OnAppended.Raise(&arg);
				return;
			}
		}
		Reserve(newcapacity);
	}
	// ��������׷��Ԫ���¼�
	T temp;
	temp = x;
	EventArg arg(this, &temp);
	this->OnAppending.Raise(&arg);

	// ������Ԫ��
	m_a[m_count++] = temp;

	// ����׷��Ԫ������¼�
	this->OnAppended.Raise(&arg);
}

template <class T>
void List<T>::Append(int count, const T* p) 
{
	int i;
	if (count > 0 && p) 
	{
		if (count + m_count > m_capacity) 
		{
			int newcapacity = NewCapacity();
			if (newcapacity < count + m_count)
				newcapacity = count + m_count;

			Reserve(newcapacity);
		}

		for (i = 0; i < count; i++) 
		{
			// ��������׷��Ԫ���¼�
			T temp;
			temp = p[i];
			EventArg arg(this, &temp);
			this->OnAppending.Raise(&arg);

			// ������Ԫ��
			m_a[m_count++] = temp;

			// ����׷��Ԫ������¼�
			this->OnAppended.Raise(&arg);
		}
	}
}

template <class T>
void  List<T>::Append(const List<T> &list)
{
	int count = list.Count();
	this->Reserve(m_count+count);

	for (int i = 0; i < count; ++i)
	{
		this->Append(list[i]);
	}
}

template <class T>
void List<T>::Insert(int i, const T& x) 
{
	if (i >= 0 && i <= m_count) 
	{
		if (m_count == m_capacity) 
		{
			int newcapacity = NewCapacity();
			Reserve( newcapacity );
		}
		DestroyElement(m_a[m_count]);
		m_count++;

		if (i < m_count-1) 
		{
			Move(i+1, i, m_count-1-i);
			memset(&m_a[i], 0, sizeof(T));
			ConstructDefaultElement(&m_a[i]);
		}
		else 
		{
			ConstructDefaultElement(&m_a[m_count-1]);
		}
		m_a[i] = x;
	}
}

template <class T>
void List<T>::Remove()
{
	Remove(m_count-1);
} 

template <class T>
void List<T>::Remove(int i)
{
	if (i >= 0 && i < m_count) 
	{
		EventArg arg(this, &m_a[i]);
		this->OnRemoving.Raise(&arg);
		DestroyElement(m_a[i]);
		this->OnRemoved.Raise(&arg);

		memset(&m_a[i], 0, sizeof(T));
		Move(i, i+1, m_count-1-i);

		memset(&m_a[m_count-1], 0, sizeof(T));
		ConstructDefaultElement(&m_a[m_count-1]);

		m_count--;
	}
} 

template <class T>
void List<T>::Clear()
{
	int i;
	for (i = m_count-1; i >= 0; i--)
	{
		DestroyElement( m_a[i] );
		memset(&m_a[i], 0, sizeof(T));
		ConstructDefaultElement(&m_a[i]);
	}
	m_count = 0;
}

template <class T>
void List<T>::Reverse()
{
	// NOTE:
	// If anything in "T" depends on the value of this's address,
	// then don't call Reverse().
	char t[sizeof(T)];
	int i = 0;  
	int j = m_count-1;
	for (; i < j; i++, j--) 
	{
		memcpy(t, &m_a[i], sizeof(T));
		memcpy(&m_a[i], &m_a[j], sizeof(T));
		memcpy(&m_a[j], t, sizeof(T));
	}
}

template <class T>
void List<T>::Swap(int i, int j)
{
	if (i != j && i >= 0 && j >= 0 && i < m_count && j < m_count) 
	{
		char t[sizeof(T)];
		memcpy(t,       &m_a[i], sizeof(T));
		memcpy(&m_a[i], &m_a[j], sizeof(T));
		memcpy(&m_a[j], t,       sizeof(T));
	}
}

template <class T>
int List<T>::Search(const T &key, CompareFunc pCompare) const
{
	for (int i = 0; i < m_count; i++) 
	{
		if (pCompare(key, m_a[i]) == 0) 
			return i;
	}
	return -1;
}

template <class T>
int List<T>::BinarySearch(T a[], int low, int heigh, const T &key, CompareFunc pCompare) const
{
	if (low > heigh)
	{
		return -1;
	}
	else
	{
		int mid = low + ((heigh - low) / 2);
		int flag = pCompare(a[mid], key);

		if (flag == 0)
			return mid;
		else if(flag < 0)
		{
			low = mid + 1;
			return BinarySearch(a, low, heigh, key, pCompare);
		}
		else
		{
			heigh = mid -1;
			return BinarySearch(a, low, heigh, key, pCompare);
		}
	}
	return -1;
}

template <class T>
int List<T>::BinarySearch(const T &key, CompareFunc pCompare) const
{
	return this->BinarySearch(m_a, 0, m_count-1, key, pCompare);
}

template <class T>
bool List<T>::HeapSort(CompareFunc pCompare)
{
	CSort<T> sortor(pCompare);
	sortor.HeapSort(m_a, m_count);
	return true;
}

template <class T>
bool List<T>::QuickSort(CompareFunc pCompare)
{
	CSort<T> sortor(pCompare);
	sortor.QuickSort(m_a, 0, m_count-1);
	return true;
}

template <class T>
bool List<T>::Permute(const int* index)
{
	bool rc = false;
	if (m_a && m_count > 0 && index) 
	{
		int i;
		T* buffer = (T*)CMemoryStrategy::OnMalloc(m_count*sizeof(buffer[0]));
		memcpy(buffer, m_a, m_count*sizeof(T));

		for (i = 0; i < m_count; i++)
			memcpy( m_a+i, buffer+index[i], sizeof(T) ); // must use memcopy and not operator=

		CMemoryStrategy::OnFree(buffer);
		rc = true;
	}
	return rc;
}

template <class T>
void List<T>::Zero()
{
	int i;
	if (m_a && m_capacity > 0) 
	{
		for (i = m_capacity-1; i >= 0; i--) 
		{
			DestroyElement(m_a[i]);
			memset(&m_a[i], 0, sizeof(T));
			ConstructDefaultElement(&m_a[i]);
		}
	}
}

template <class T>
void List<T>::Reserve(int newcap) 
{
	if (m_capacity < newcap)
		SetCapacity(newcap);
}

template <class T>
void List<T>::Shrink()
{
	SetCapacity(m_count);
}

template <class T>
void List<T>::Destroy()
{
	SetCapacity(0);
}

// low level memory managment //////////////////////////////////////////

template <class T>
void List<T>::SetCount(int count) 
{
	if (count >= 0 && count <= m_capacity)
		m_count = count;
}

template <class T>
void List<T>::SetCapacity(int capacity) 
{
	// uses "placement" for class construction/destruction
	int i;
	if (capacity < 1) 
	{
		if (m_a != NULL)
		{
			for (i = m_capacity-1; i >= 0; i--) 
			{
				DestroyElement(m_a[i]);
			}
			Realloc(m_a,0);
			m_a = 0;
		}
		m_count = 0;
		m_capacity = 0;
	}
	else if (m_capacity < capacity)
	{
		// growing
		m_a = Realloc(m_a, capacity);
		// initialize new elements with default constructor
		if (0 != m_a)
		{
			memset(m_a + m_capacity, 0, (capacity-m_capacity)*sizeof(T));
			for (i = m_capacity; i < capacity; i++) 
			{
				ConstructDefaultElement(&m_a[i]);
			}
			m_capacity = capacity;
		}
		else
		{
			// memory allocation failed
			m_capacity = 0;
			m_count = 0;
		}
	}
	else if (m_capacity > capacity) 
	{
		// shrinking
		for (i = m_capacity-1; i >= capacity; i--) 
		{
			DestroyElement(m_a[i]);
		}
		if (m_count > capacity)
			m_count = capacity;

		m_capacity = capacity;
		m_a = Realloc(m_a, capacity);
		if (0 == m_a)
		{
			// memory allocation failed
			m_capacity = 0;
			m_count = 0;
		}
	}
}

#if defined(ON_COMPILER_MSC)
#pragma warning(pop)
#endif

#endif //COMMONEXT_LISTIMPLEMENT