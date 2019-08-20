
#ifndef COMMONEXT_MEMORYSTRATEGY
#define COMMONEXT_MEMORYSTRATEGY

/// <summary>
/// �����ڴ�ķ�����ͷ�
/// </summary>
class __declspec(dllexport) CMemoryStrategy
{
public:

	/// <summary>
	/// ��ϵͳ�������ָ��size���ֽڵ��ڴ�ռ䣬����һ��ָ�������ʼ��ַ��ָ�롣
	/// </summary>
	/// <remarks>������ڴ�δִ���κγ�ʼ�������ܰ����������ݡ�</remarks>
	/// <param name="size">[I] ָ��Ҫ������ڴ�Ĵ�С��</param>
	/// <returns>���ڴ����ɹ����򷵻�һ��ָ�������ʼ��ַ��ָ�룻���򷵻� NULL��</returns>
	/// <author name="hps" date="2012.6.27   16:28"/>
	static void* OnMalloc(size_t size);

	/// <summary>
	/// ���ڴ�Ķ�̬�洢���з���n������Ϊsize�������ռ䣬����һ��ָ�������ʼ��ַ��ָ�롣
	/// </summary>
	/// <remarks>�ڶ�̬�������ڴ���Զ���ʼ�����ڴ�ռ�Ϊ�㡣</remarks>
	/// <param name="n">[I] ָ��Ҫ������ڴ��������</param>
	/// <param name="size">[I] ָ���ڴ���䵥Ԫ�Ĵ�С��</param>
	/// <returns>���ڴ����ɹ����򷵻�һ��ָ�������ʼ��ַ��ָ�룻���򷵻� NULL��</returns>
	/// <author name="hps" date="2012.6.27   16:30"/>
	static void* OnCalloc(size_t n, size_t size);

	/// <summary>
	/// �ͷ� memblock ָ����ڴ�ռ�����ݡ�
	/// </summary>
	/// <remarks></remarks>
	/// <param name="memblock">[I] ָ��Ҫ�ͷ���Դ��ָ�롣</param>
	/// <returns></returns>
	/// <author name="hps" date="2012.6.27   16:34"/>
	static void OnFree(void* memblock);

	/// <summary>
	/// �ͷ� memblock ָ����ڴ�ռ����Դ�������·����СΪ size ���ڴ�ռ䡣
	/// </summary>
	/// <remarks></remarks>
	/// <param name="memblock">[I] ָ��Ҫ���·�����Դ��ָ�롣</param>
	/// <param name="size">[I] ָ�����·�����Դ�Ĵ�С��</param>
	/// <returns>���ڴ����ɹ����򷵻�һ��ָ�������ʼ��ַ��ָ�룻���򷵻� NULL��</returns>
	/// <author name="hps" date="2012.6.27   16:36"/>
	static void* OnRealloc(void *memblock, size_t size);
};

#endif // COMMONEXT_MEMORYSTRATEGY
