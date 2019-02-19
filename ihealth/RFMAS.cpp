#include "RFMAS.h"

RFMAS::RFMAS(void)
{
	init();
	Reset();
}

RFMAS::~RFMAS(void)
{
}


void RFMAS::Next()
{
	if (m_questions.size() > m_current_index) {
		m_current_index++;
	}
}

void RFMAS::Prev()
{
	if (m_current_index > 0) {
		m_current_index--;
	}
}

void RFMAS::Reset()
{
	m_current_index = 0;
	m_result.clear();
}


int RFMAS::getScore()
{
	int score = 0;
	std::map<int, int>::iterator begin = m_result.begin();
	for (; begin != m_result.end(); begin++) {
		score += begin->second + 1;	
	}

	return score;
}

void RFMAS::saveAnswer(int i , int index)
{
	m_result[i] = index;
}

void RFMAS::getQuestionAndAnswer(std::vector<std::wstring>& questions, std::vector<std::wstring>& answers)
{
	std::wstring question;
	std::wstring answer;

	std::map<int, int>::iterator begin = m_result.begin();
	for (; begin != m_result.end(); begin++) {
		question = getQuestion(begin->first);
		answer = getAnswers(begin->first).at(begin->second);

		questions.push_back(question);
		answers.push_back(answer);
	}
}

std::wstring RFMAS::getQuestion(int i)
{
	return m_questions.at(i);
}

std::vector<std::wstring> RFMAS::getAnswers(int i)
{
	return m_answers.at(i);
}

void RFMAS::init()
{
	m_questions.push_back(_T("������λ��������λ"));
	std::vector<std::wstring> answers;
	answers.push_back(_T("1.�Լ�ǣ������(��ʼλ��������,����ϥ,�����Լ��ý�����ǣ���򻼲���,�ý��Ȱ��������ƶ�)"));
	answers.push_back(_T("2.��֫�����������°�����֮�ƶ�(��ʼλͬ��,��֫���ں���) "));
	answers.push_back(_T("3.�ý�����֫��������֫�������,��֫�����ƶ������������ƶ���(��ʼλͬ��)"));
	answers.push_back(_T("4.������֫�����ƶ����Բ࣬����������λ��֮�˶���(��ʼλͬ��) "));
	answers.push_back(_T("5.�ƶ�����֫����������λ����ƽ��(��ʼλͬ�ϣ���ǰ�죬��֫ǰ��)"));
	answers.push_back(_T("6.��3s�ڷ�����ԡ�(��ʼλͬ�ϣ�������) "));
	m_answers.push_back(answers);

	m_questions.push_back(_T("������λ��������"));
	answers.clear();
	answers.push_back(_T("1.���ԣ�ͷ��̧�𣬵�������(�������߲���)"));
	answers.push_back(_T("2.�Ӳ��Ե���������(���������ƶ����������̻����ܿ���ͷ������)"));
	answers.push_back(_T("3.�Ӳ��Ե���������(׼����ʱ������������֫��������)"));
	answers.push_back(_T("4.�Ӳ��Ե���������(����Ҫ����)"));
	answers.push_back(_T("5.���Ե���������(����Ҫ����)"));
	answers.push_back(_T("6.��10s�ڴ����Ե���������(����Ҫ����)"));
	m_answers.push_back(answers);

	m_questions.push_back(_T("��λƽ��"));
	answers.clear();
	answers.push_back(_T("1.������֧�ֲ�����������������"));
	answers.push_back(_T("2.��֧������10s�����÷��֣�˫ϥ��˫�㿿£��˫����ŵؿ�£��"));
	answers.push_back(_T("3.��֧�������������ܺܺõ�ǰ�ƣ��ҷ�����ȣ�������˫�Ŵ��ܺܺõ�ǰ�ƣ�ͷ����չ��������ȳ��أ�"));
	answers.push_back(_T("4.��֧����������ת��ͷ��������󿴣�˫���ŵ�֧�֣�����˫����չ��˫���ƶ���˫�ַ��ڴ����ϣ���Ҫ�ƶ��������ϣ�"));
	answers.push_back(_T("5.��֧����������ǰ�����沢����ԭλ��˫���ŵأ���������׽ס�������Ⱥ�˫�㲻Ҫ�ƶ�����Ҫʱ֧�ֻ��ۣ������ٱ��봥����ǰ10cm�ĵ��棩"));
	answers.push_back(_T("6.��֧�������ڵ����ϣ������෽���沢����ԭλ��Ҫ������ͬ�ϣ������߱������λ��������ǰ��������"));
	m_answers.push_back(answers);

	m_questions.push_back(_T("������վ"));
	answers.clear();
	answers.push_back(_T("1.��Ҫ���˰���վ���κη�����"));
	answers.push_back(_T("2.���ڱ���׼����ʱ������վ�����طֲ����������ַ��֣�"));
	answers.push_back(_T("3.����վ�𣨲��������طֲ����������ַ��֣�"));
	answers.push_back(_T("4.����վ�𣬲���ֱ�ź�ϥά��5�루���������طֲ�������"));
	answers.push_back(_T("5.����վ�����������׼����ʱ���������������طֲ���������ȫ��ֱ�ź�ϥ��"));
	answers.push_back(_T("6.����վ�����������׼����ʱ����������10�����ظ����Σ����������طֲ�������"));
	m_answers.push_back(answers);

	m_questions.push_back(_T("����"));
	answers.clear();
	answers.push_back(_T("1.���û���վ����һ����ǰ���������ص��Źؽڱ�����չ����׼����ʱ���������"));
	answers.push_back(_T("2.��һ����׼��������԰�����������"));
	answers.push_back(_T("3.��������ܶ������ߣ�������κθ������ߣ�3m"));
	answers.push_back(_T("4.���ø�������15s���ܶ�������5m"));
	answers.push_back(_T("5.���ø�������25s���ܶ�������10m��Ȼ��ת��ʰ�����һ��Сɳ������������һֻ�֣������߻�ԭ��"));
	answers.push_back(_T("6.35s�������ļ�̨��3�Σ����û��ø������ߣ������ܷ����ˣ�"));
	m_answers.push_back(answers);

	m_questions.push_back(_T("��֫����"));
	answers.clear();
	answers.push_back(_T("1.��λ���Ͼ���֫����չ��ؽڣ�����ǰ��������Ҫ���λ�ò�����֧�֣�ʹ����ֱ��"));
	answers.push_back(_T("2.��λ�������Ͼ���ֱ����֫2s����������֫������Ҫ���λ�ã����߱���ʹ��֫���������������ֱ��20�ȷ�Χ�ڣ�"));
	answers.push_back(_T("3.��֫��λͬ2�������ⲿʹ���Ƽ�ʱ�뿪ǰ����԰���ǰ������ "));
	answers.push_back(_T("4.��λ��ʹ��֫��ֱǰ��90�ȱ���2s��������֫�����������⣬����������ʼ磩"));
	answers.push_back(_T("5.��λ�����߾ٱ�ͬ4��ǰ��90�ȣ���ά��10sȻ��ԭ�����߱���ά����֫��������������������"));
	answers.push_back(_T("6.վ�����ֵ�ǽ��������ת��ǽʱҪά����֫��λ�ã���֫��չ90�㣬����ƽѹ��ǽ�ϣ�"));
	m_answers.push_back(answers);

	m_questions.push_back(_T("�ֵ��˶�"));
	answers.clear();
	answers.push_back(_T("1.��λ�������û����������ԣ�ǰ���������ϣ���Բ������ڻ������У�Ҫ�������� �����е�����������棬���������⣩"));
	answers.push_back(_T("2.��λ�������ƫ�ƣ�������ǰ�۳߲࿿�ţ�������ǰ�������λ��Ĵָ��ǰ�۳�һֱ�ߣ�����������Բ���壬Ȼ��Ҫ���߽���̧�����棬�������������ǰ��"));
	answers.push_back(_T("3.��λ�����������ԣ���ǰ�������ⲻҪ֧�֣�����ֱ��λ��3�M4�ķ�Χ���ɣ�"));
	answers.push_back(_T("4.��ǰ�죬��˫�ּ���һֱ��14cm�Ĵ��򣬲��������£���Ӧ�÷��������뻼�߽�Զ�ĵط���ʹ������ȫ��ֱ˫�۲����õ��򣬼����ǰ�죬˫����ֱ������������ֱ��˫��Ҫ�Ӵ���"));
	answers.push_back(_T("5.����������һ�����ϱ�������������������һ������ϣ����ܸı䱭�ӵ���̬��"));
	answers.push_back(_T("6.������Ĵָ��ÿһ����ָ��ָ��10s����14�����ϣ���ʳָ��ʼ��ÿ����ָ������Ĵָ������Ĵָ��һ����ָ������һ����ָ���������"));
	m_answers.push_back(answers);

	m_questions.push_back(_T("�ֵľ�ϸ����"));
	answers.clear();
	answers.push_back(_T("1.����һ���ֱ�ñ���ٷ��£�������ǰ��ۣ������ñ���ڿ�������������ϣ�"));
	answers.push_back(_T("2.�ӱ�������һ���Ƕ���Ȼ�������һ���������������8���Ƕ����������ӱ��������֫���쵽�����������Ҳ౭��Ķ�������౭�"));
	answers.push_back(_T("3.������ˮƽ��ֹ�ڴ�ֱ���ϣ�20s����10�Σ�����Ҫ��5������������ֹ�ڴ�ֱ���ϣ�"));
	answers.push_back(_T("4.��һֻǦ����ֽ������Ѹ�ٵĵ�㣨����������ÿ���ӵ������㣬����5s��������д��һ���ñʣ���㲻���ã�"));
	answers.push_back(_T("5.��һ��Һ�������У������ͷȥӭ�ͳף�����Һ�������"));
	answers.push_back(_T("6.��������ͷ�󲿵�ͷ��"));
	m_answers.push_back(answers);

	m_questions.push_back(_T("ȫ������"));
	answers.clear();
	answers.push_back(_T("1.�ٻ��������ƶ����岿��ʱ��������"));
	answers.push_back(_T("2.�ƶ����岿��ʱ�ɸо���һЩ��Ӧ��"));
	answers.push_back(_T("3.�仯��������ʱ�ٻ���������ʱ��������������ʱ�����ߡ�"));
	answers.push_back(_T("4.��������״̬��"));
	answers.push_back(_T("5.50��ʱ�伡�����ߡ�"));
	answers.push_back(_T("6.���������������ߡ�"));
	m_answers.push_back(answers);
}

RFFMA::RFFMA()
{
	init();
}

RFFMA::~RFFMA()
{

}

void RFFMA::init()
{
	m_current_index = 0;
	m_questions.push_back(_T("�Ŷ�ͷ��"));
	std::vector<std::wstring> answers;
	answers.push_back(_T("����������"));
	answers.push_back(_T("/"));
	answers.push_back(_T("��������"));
	m_answers.push_back(answers);

	m_questions.push_back(_T("����ͷ��"));
	answers.clear();
	answers.push_back(_T("����������"));
	answers.push_back(_T("/"));
	answers.push_back(_T("��������"));
	m_answers.push_back(answers);

	m_questions.push_back(_T("������"));
	answers.clear();
	answers.push_back(_T("��ȫ���ܽ���"));
	answers.push_back(_T("�������"));
	answers.push_back(_T("��ͣ�ٵس�����"));
	m_answers.push_back(answers);

	m_questions.push_back(_T("�����"));
	answers.clear();
	answers.push_back(_T("��ȫ���ܽ���"));
	answers.push_back(_T("�������"));
	answers.push_back(_T("��ͣ�ٵس�����"));
	m_answers.push_back(answers);

	m_questions.push_back(_T("����չ��90��"));
	answers.clear();
	answers.push_back(_T("��ȫ���ܽ���"));
	answers.push_back(_T("�������"));
	answers.push_back(_T("��ͣ�ٵس�����"));
	m_answers.push_back(answers);

	m_questions.push_back(_T("������"));
	answers.clear();
	answers.push_back(_T("��ȫ���ܽ���"));
	answers.push_back(_T("�������"));
	answers.push_back(_T("��ͣ�ٵس�����"));
	m_answers.push_back(answers);

	m_questions.push_back(_T("������"));
	answers.clear();
	answers.push_back(_T("��ȫ���ܽ���"));
	answers.push_back(_T("�������"));
	answers.push_back(_T("��ͣ�ٵس�����"));
	m_answers.push_back(answers);

	m_questions.push_back(_T("ǰ������"));
	answers.clear();
	answers.push_back(_T("��ȫ���ܽ���"));
	answers.push_back(_T("�������"));
	answers.push_back(_T("��ͣ�ٵس�����"));
	m_answers.push_back(answers);

	m_questions.push_back(_T("�����ա�����"));
	answers.clear();
	answers.push_back(_T("��ȫ���ܽ���"));
	answers.push_back(_T("�������"));
	answers.push_back(_T("��ͣ�ٵس�����"));
	m_answers.push_back(answers);

	m_questions.push_back(_T("����չ"));
	answers.clear();
	answers.push_back(_T("��ȫ���ܽ���"));
	answers.push_back(_T("�������"));
	answers.push_back(_T("��ͣ�ٵس�����"));
	m_answers.push_back(answers);

	m_questions.push_back(_T("ǰ����ǰ"));
	answers.clear();
	answers.push_back(_T("��ȫ���ܽ���"));
	answers.push_back(_T("�������"));
	answers.push_back(_T("��ͣ�ٵس�����"));
	m_answers.push_back(answers);

	m_questions.push_back(_T("�ִ���׵"));
	answers.clear();
	answers.push_back(_T("û�����Ի"));
	answers.push_back(_T("�ֽ������Խ����ǰ�ϼ�"));
	answers.push_back(_T("��˳�����"));
	m_answers.push_back(answers);

	m_questions.push_back(_T("��ؽ�����90�ȣ�ǰ����ǰ������"));
	answers.clear();
	answers.push_back(_T("��ʼʱ�ֱ�������չ����ؽ�����"));
	answers.push_back(_T("�ڽӽ��涨λ��ʱ��ؽ���չ����ؽ�����"));
	answers.push_back(_T("��˳��������"));
	m_answers.push_back(answers);

	m_questions.push_back(_T("��0�ȣ�����90��,ǰ����ǰ������"));
	answers.clear();
	answers.push_back(_T("���������ǰ��,������ǰ"));
	answers.push_back(_T("�硢��λ��ȷ����������ǰ������"));
	answers.push_back(_T("˳�����"));
	m_answers.push_back(answers);

	m_questions.push_back(_T("��ؽ���չ90�ȣ�����ֱ��ǰ����ǰ "));
	answers.clear();
	answers.push_back(_T("��ʼʱ���������ǰ��ƫ�뷽������ǰ"));
	answers.push_back(_T("������ɶ�������ؽ�������ǰ�۲�����ǰ"));
	answers.push_back(_T("/"));
	m_answers.push_back(answers);

	m_questions.push_back(_T("��ؽ�ǰ���ٱ۹�ͷ����ֱǰ������λ"));
	answers.clear();
	answers.push_back(_T("��ʼʱ��ؽ��������ؽ���չ"));
	answers.push_back(_T("��������;����ؽ���������ؽ���չ"));
	answers.push_back(_T("/"));
	m_answers.push_back(answers);

	m_questions.push_back(_T("������30�ȣ�90������ֱǰ����ǰ����"));
	answers.clear();
	answers.push_back(_T("ǰ����ǰ������ȫ���ܻ����λ����ȷ"));
	answers.push_back(_T("����λ����ȷ�����������ǰ����"));
	answers.push_back(_T("/"));
	m_answers.push_back(answers);

	m_questions.push_back(_T("���Ŷ�ͷ��������ͷ����ָ����3����"));
	answers.clear();
	answers.push_back(_T("����2��3���������Կ���"));
	answers.push_back(_T("1���������Կ���������2�������Ծ"));
	answers.push_back(_T("��Ծ�����1��,���޷��俺��"));
	m_answers.push_back(answers);

	m_questions.push_back(_T("��0�ȣ�����90������"));
	answers.clear();
	answers.push_back(_T("���ܱ�����ؽڴ�15��"));
	answers.push_back(_T("����������������ܿ�������"));
	answers.push_back(_T("ʩ����΢�����Կɱ�������"));
	m_answers.push_back(answers);

	m_questions.push_back(_T("��0�ȣ�����90��������"));
	answers.clear();
	answers.push_back(_T("������������"));
	answers.push_back(_T("������ȫ�ؽڷ�Χ���������ؽ�"));
	answers.push_back(_T("��ͣ�ٽ���"));
	m_answers.push_back(answers);

	m_questions.push_back(_T("����"));
	answers.clear();
	answers.push_back(_T("���ܱ�����ؽڴ�15��"));
	answers.push_back(_T("����������������ܿ�������"));
	answers.push_back(_T("ʩ����΢�����ɱ�������"));
	m_answers.push_back(answers);

	m_questions.push_back(_T("������"));
	answers.clear();
	answers.push_back(_T("������������"));
	answers.push_back(_T("������ȫ�ؽڷ�Χ���������ؽ�"));
	answers.push_back(_T("��ƽ����ͣ�ٵĽ���"));
	m_answers.push_back(answers);

	m_questions.push_back(_T("�����˶�"));	
	answers.clear();
	answers.push_back(_T("���ܽ���"));
	answers.push_back(_T("���������ȫ"));
	answers.push_back(_T("�������"));
	m_answers.push_back(answers);

	m_questions.push_back(_T("��������"));
	answers.clear();
	answers.push_back(_T("��������"));
	answers.push_back(_T("��������������ܷ���������������ָ"));
	answers.push_back(_T("�����������������չ"));
	m_answers.push_back(answers);

	m_questions.push_back(_T("������չ"));
	answers.clear();
	answers.push_back(_T("������չ"));
	answers.push_back(_T("�ܷ���������������ָ "));
	answers.push_back(_T("����ȫ������չ"));
	m_answers.push_back(answers);

	m_questions.push_back(_T("��״ץ��"));
	answers.clear();
	answers.push_back(_T("���ܱ���Ҫ��λ��"));
	answers.push_back(_T("����΢��"));
	answers.push_back(_T("�ֿܵ��൱������"));
	m_answers.push_back(answers);

	m_questions.push_back(_T("����"));
	answers.clear();
	answers.push_back(_T("��ȫ���� "));
	answers.push_back(_T("����΢��"));
	answers.push_back(_T("�ֿܵ��൱������"));
	m_answers.push_back(answers);

	m_questions.push_back(_T("Բ��״ץ��"));
	answers.clear();
	answers.push_back(_T("���ܱ���Ҫ��λ��"));
	answers.push_back(_T("����΢��"));
	answers.push_back(_T("�ֿܵ��൱������"));
	m_answers.push_back(answers);

	m_questions.push_back(_T("����ץ��"));
	answers.clear();
	answers.push_back(_T("���ܱ���Ҫ��λ��"));
	answers.push_back(_T("����΢��"));
	answers.push_back(_T("�ֿܵ��൱������"));
	m_answers.push_back(answers);

	m_questions.push_back(_T("���"));
	answers.clear();
	answers.push_back(_T("�������"));
	answers.push_back(_T("������"));
	answers.push_back(_T("�����"));
	m_answers.push_back(answers);

	m_questions.push_back(_T("����ϰ�"));
	answers.clear();
	answers.push_back(_T("���Ի򲻹���"));
	answers.push_back(_T("��Ȼ����"));
	answers.push_back(_T("��"));
	m_answers.push_back(answers);

	m_questions.push_back(_T("�ٶ�"));
	answers.clear();
	answers.push_back(_T("�Ͻ��೤6��"));
	answers.push_back(_T("�Ͻ��೤2��5��"));
	answers.push_back(_T("������<2��"));
	m_answers.push_back(answers);
}

void RFFMA::Next()
{
	if (m_questions.size() > m_current_index) {
		m_current_index++;
	}
}

void RFFMA::Prev()
{
	if (m_current_index > 0) {
		m_current_index--;
	}
}

void RFFMA::Reset()
{
	m_current_index = 0;
	m_result.clear();
}

int RFFMA::getScore()
{
	int score = 0;
	std::map<int, int>::iterator begin = m_result.begin();
	for (; begin != m_result.end(); begin++) {
		score += begin->second + 1;	
	}

	return score;
}

void RFFMA::saveAnswer(int i , int index)
{
	m_result[i] = index;
}


void RFFMA::getQuestionAndAnswer(std::vector<std::wstring>& questions, std::vector<std::wstring>& answers)
{
	std::wstring question;
	std::wstring answer;

	std::map<int, int>::iterator begin = m_result.begin();
	for (; begin != m_result.end(); begin++) {
		question = getQuestion(begin->first);
		answer = getAnswers(begin->first).at(begin->second);

		questions.push_back(question);
		answers.push_back(answer);
	}
}

std::wstring RFFMA::getQuestion(int i)
{
	return m_questions.at(i);
}

std::vector<std::wstring> RFFMA::getAnswers(int i)
{
	return m_answers.at(i);
}