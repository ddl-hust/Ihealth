#pragma once

#ifdef COMMONLIB_MODULE
#define COMMONLIBDLLIMPEXP __declspec(dllexport)
#else
#define COMMONLIBDLLIMPEXP __declspec(dllimport)
#endif

#include "TreeNode.h"
#include "TreeIter.h"

namespace Ext
{

	namespace Collection
	{
		// ��һ��·���ϵĺڽڵ����Ŀ����Ӧ��������ɫ�ڵ����Ŀ���������ɫ���ڱ�ʶ NullNode
		enum EColor 
		{ 
			Red		= 1,
			Black	= 2,
			Green	= 4,
			Blue	= 8,
			BlackGreen = 6,		// Black | Green
			BlackBlue  = 10,	// Black | Blue
		};

		template<typename K, typename V>
		class __declspec(dllexport) CRBNode : public CTreeNode<K, V>
		{
		public:  
			CRBNode()  
			{  
				//m_Color = Black;
				m_pRight = NULL;  
				m_pLeft = NULL;  
				m_pParent = NULL;  
			}  
			K m_Key;
			V m_Value;

			CRBNode* m_pParent;
			CRBNode* m_pLeft;
			CRBNode* m_pRight;
			EColor m_Color;

			virtual const K& GetKey() { return m_Key; }
			virtual V& GetValue() { return m_Value; }
			virtual CTreeNode<K, V>* GetParent() { return m_pParent; }
			virtual CTreeNode<K, V>* GetLeft() { return m_pLeft; }
			virtual CTreeNode<K, V>* GetRight() { return m_pRight; }
			virtual bool IsNull() { return m_Color == BlackGreen; }
		};

		template<typename K, typename V>
		class __declspec(dllexport) CRBTree
		{
		public:
			typedef int(*CompareFunc)(const K &a, const K &b);

		public:
			CRBTree(const CRBTree& input)
			{
				this->Initial();
				this->m_pComparor = input.m_pComparor;
				*this = input;
			}

			const CRBTree& operator= (const CRBTree& input)
			{
				if (!this->IsEmpty())
					this->Clear();

				CTreeIter<K, V> iter = input.Iter(EIterOrder::PostOrder);
				for (iter.Start(); iter.Keepup(); iter.Step())
					this->Insert(iter.GetKey(), iter.GetValue());

				return *this;
			}

		protected:
			CRBNode<K, V> *NullNode; // �ڱ��ڵ㣬��ʾ�սڵ��Ҷ�ӽڵ�
			CRBNode<K, V> *m_pRoot;
			CompareFunc m_pComparor;
			long m_NodeCount;

		public:  
			CRBTree()  
			{  
				this->Initial();
				this->m_pComparor = Ext::Collection::CompareAB;
			}

			CRBTree(CompareFunc pComparor)  
			{  
				this->Initial();
				this->m_pComparor = pComparor;
			}

			virtual ~CRBTree()  
			{  
				Destroy();
			}

			bool IsEmpty() const
			{  
				if(this->m_pRoot == NullNode)  
					return true;
				else  
					return false;
			}

			void AsEmpty() const
			{
				this->m_pRoot = NullNode;
			}

			long GetNodeCount() const
			{
				return this->m_NodeCount;
			}

			bool Find(K key, V &outputValue) const
			{
				CRBNode<K, V> *pNode = this->FindNode(key);
				if (pNode == NullNode)
					return false;

				outputValue = pNode->m_Value;
				return true;
			}

			bool Exits(K key) const
			{
				CRBNode<K, V> *pNode = this->FindNode(key);
				if (pNode == NullNode)
					return false;

				return true;
			}

			// ʹ�ü�������ȡ������ֵ������������ʱ���������¼��������Ĭ��ֵ
			// ��Ҫȷ���������¼�����ʹ�� Exits(...)��Find(...) ��ѯ���Ƿ����
			V& operator[](const K& key)
			{
				CRBNode<K, V> *pNode = this->FindNode(key);
				if (pNode != NullNode)
					return pNode->m_Value;

				V defaultVal;
				this->Insert(key, defaultVal);

				pNode = this->FindNode(key);
				return pNode->m_Value;
			}

			virtual void Insert(const K &key, const V &val) 
			{
				this->InsertNode(key, val);
			}

			virtual bool Delete(const K &key)
			{  
				CRBNode<K, V> *pNode = FindNode(key);
				if (pNode == NullNode)
					return false;

				if (pNode->m_pLeft != NullNode && pNode->m_pRight != NullNode)  
				{  
					CRBNode<K, V> *successor = InOrderSuccessor(pNode);  
					pNode->m_Value = successor->m_Value;  
					pNode->m_Key = successor->m_Key;  
					pNode = successor;
				}

				CRBNode<K, V> *pChild;  
				if (pNode->m_pRight != NullNode)  
				{  
					pChild = pNode->m_pRight;  
				}  
				else if (pNode->m_pLeft != NullNode)  
				{  
					pChild = pNode->m_pLeft;  
				}  
				else  
				{  
					pChild = NullNode;  
				}  
				pChild->m_pParent = pNode->m_pParent;  
				if (pNode->m_pParent == NullNode) 
				{  
					m_pRoot = pChild;
					NullNode->m_pParent = m_pRoot;
					NullNode->m_pLeft = m_pRoot;
					NullNode->m_pRight = m_pRoot;
				}  
				else if (pNode == pNode->m_pParent->m_pRight)  
				{  
					pNode->m_pParent->m_pRight = pChild;  
				}  
				else  
				{  
					pNode->m_pParent->m_pLeft = pChild;  
				}  
				if (pNode->m_Color == Black && !(pChild == NullNode && pChild->m_pParent == NullNode))  
				{  
					DeleteFixUp(pChild);
				}

				pNode->m_pParent = NULL;
				pNode->m_pLeft = NULL;
				pNode->m_pRight = NULL;
				delete pNode;
				pNode = NULL;

				--m_NodeCount;
				return true;  
			}  

			virtual void DeleteFixUp(CRBNode<K, V> *pNode)  
			{
				while (pNode != m_pRoot && pNode->m_Color == Black)  
				{  
					if (pNode == pNode->m_pParent->m_pLeft)
					{  
						CRBNode<K, V> *brother = pNode->m_pParent->m_pRight;  
						if (brother->m_Color == Red)   //���1��x���ֵ�w�Ǻ�ɫ�ġ�  
						{  
							brother->m_Color = Black;  
							pNode->m_pParent->m_Color = Red;  
							RotateLeft(pNode->m_pParent);  
						}  
						else      
						{  
							//���2��x���ֵ�w�Ǻ�ɫ�ģ�
							if ((brother->m_pLeft->m_Color & Black) != 0 && (brother->m_pRight->m_Color & Black) != 0)
							{  
								//��w���������Ӷ��Ǻ�ɫ�ġ�
								brother->m_Color = Red;  
								pNode = pNode->m_pParent;  
							}  
							else if((brother->m_pRight->m_Color & Black) != 0)
							{  
								//���3��x���ֵ�w�Ǻ�ɫ�ģ�w���Һ����Ǻ�ɫ��w�������Ǻ�ɫ����  
								brother->m_Color = Red;  
								brother->m_pLeft->m_Color = Black;  
								RotateRight(brother);
							}  
							else if(brother->m_pRight->m_Color == Red)  
							{  
								//���4��x���ֵ�w�Ǻ�ɫ�ģ���w���Һ���ʱ��ɫ�ġ�  
								brother->m_Color = pNode->m_pParent->m_Color;  
								pNode->m_pParent->m_Color = Black;  
								brother->m_pRight->m_Color = Black;  
								RotateLeft(pNode->m_pParent);  
								pNode = m_pRoot;  
							}  
						}  
					}  
					else  
					{  
						//������������������1�У�node��Ϊ�Һ��Ӷ������ġ�  
						//22        else (same as then clause with "m_pRight" and "m_pLeft" exchanged)  
						//ͬ����ԭ��һ�£�ֻ������������Ϊ����������������Ϊ���������ɡ��������벻�䡣  
						CRBNode<K, V>* brother = pNode->m_pParent->m_pLeft;  
						if (brother->m_Color == Red)  
						{  
							brother->m_Color = Black;  
							pNode->m_pParent->m_Color = Red;  
							RotateRight(pNode->m_pParent);  
						}  
						else  
						{  
							if ((brother->m_pLeft->m_Color & Black) != 0 && (brother->m_pRight->m_Color & Black) != 0)
							{  
								brother->m_Color = Red;  
								pNode = pNode->m_pParent;  
							}  
							else if ((brother->m_pLeft->m_Color & Black) != 0)
							{  
								brother->m_Color = Red;  
								brother->m_pRight->m_Color = Black;  
								RotateLeft(brother);  
							}  
							else if (brother->m_pLeft->m_Color == Red)  
							{  
								brother->m_Color = pNode->m_pParent->m_Color;  
								pNode->m_pParent->m_Color = Black;  
								brother->m_pLeft->m_Color = Black;  
								RotateRight(pNode->m_pParent);  
								pNode = m_pRoot;  
							}  
						}  
					}  
				}  
				NullNode->m_pParent = m_pRoot;   //���node��Ϊ����㣬  
				pNode->m_Color = Black;    //����Ϊ��ɫ

				if (pNode == NullNode)
					pNode->m_Color = BlackGreen;
			}  

			//��������ʵ��  
			CRBNode<K, V>* RotateLeft(CRBNode<K, V> *pNode)  
			{  
				if (pNode == NullNode || pNode->m_pRight == NullNode)    
					return pNode;

				CRBNode<K, V>* lowerRight = pNode->m_pRight;  
				lowerRight->m_pParent =  pNode->m_pParent;  
				pNode->m_pRight = lowerRight->m_pLeft;  
				if (lowerRight->m_pLeft != NullNode)  
				{  
					lowerRight->m_pLeft->m_pParent = pNode;  
				}  
				if (pNode->m_pParent == NullNode) //pNode�Ǹ��ڵ�
				{  
					m_pRoot = lowerRight;
					this->ResetNullNode(m_pRoot);
				}  
				else  
				{  
					if (pNode->IsLeftChild() == true)
						pNode->m_pParent->m_pLeft = lowerRight;  
					else  
						pNode->m_pParent->m_pRight = lowerRight;
				}  
				pNode->m_pParent = lowerRight;  
				lowerRight->m_pLeft = pNode;

				return lowerRight;
			}  

			//��������ʵ��  
			CRBNode<K, V>* RotateRight(CRBNode<K, V> *pNode)  
			{  
				if (pNode == NullNode || pNode->m_pLeft == NullNode)
					return pNode;

				CRBNode<K, V> *lowerLeft = pNode->m_pLeft;  
				pNode->m_pLeft = lowerLeft->m_pRight;  
				lowerLeft->m_pParent = pNode->m_pParent;  
				if (lowerLeft->m_pRight != NullNode)
					lowerLeft->m_pRight->m_pParent = pNode;  

				if (pNode->m_pParent == NullNode) //pNode�Ǹ��ڵ�
				{  
					m_pRoot = lowerLeft;
					this->ResetNullNode(m_pRoot);
				}  
				else  
				{  
					if (pNode->IsLeftChild() == true)
						pNode->m_pParent->m_pLeft = lowerLeft;
					else
						pNode->m_pParent->m_pRight = lowerLeft;
				}  
				pNode->m_pParent = lowerLeft;  
				lowerLeft->m_pRight = pNode;

				return lowerLeft;
			}

			//  
			inline CRBNode<K, V>* Predecessor(CRBNode<K, V> *pNode)  
			{  
				if (pNode == NullNode)
					return NullNode;  

				CRBNode<K, V> *result = pNode->m_pLeft;      //��pNode�ڵ����Һ���ʱ�����������������Ľڵ� 
				while (result != NullNode)
				{  
					if (result->m_pRight != NullNode)
						result = result->m_pRight;			
					else
						break;  
				}

				if (result == NullNode)  
				{
					// ��pNode��������Ϊ�գ�һֱ���ϣ�ֱ����һ�γ���һ���ڵ㣨p�����丸�ڵ���ҽڵ㣬��p.Parent����pNode��ǰ�̽ڵ�
					CRBNode<K, V>* index = pNode->m_pParent;  
					result = pNode;  
					while (index != NullNode && result == index->m_pLeft)  
					{
						result = index;
						index = index->m_pParent;
					}
					result = index;
				}  
				return result;  
			}  

			inline CRBNode<K, V>* InOrderSuccessor(CRBNode<K, V> *pNode)
			{  
				if (pNode == NullNode)
					return NullNode;

				CRBNode<K, V> *result = pNode->m_pRight;   //��pNode�ڵ����Һ���ʱ����������������С�Ľڵ�  
				while (result != NullNode)
				{  
					if (result->m_pLeft != NullNode)
						result = result->m_pLeft;
					else
						break;
				}

				if (result == NullNode)  
				{  
					// ��pNode��������Ϊ�գ�һֱ���ϣ�ֱ����һ�γ���һ���ڵ㣨p�����丸�ڵ����ڵ㣬��p.Parent����pNode�ĺ�̽ڵ�
					CRBNode<K, V> *parent = pNode->m_pParent;
					result = pNode;  
					while (parent != NullNode && result == parent->m_pRight)  
					{  
						result = parent;  
						parent = parent->m_pParent;  
					}  
					result = parent;
				}  
				return result;  
			}

			CTreeIter<K, V> Iter(EIterOrder::Enums order) const
			{
				//CTreeIter<K, V> iter(this->m_pRoot, order);
				//return iter;
				return this->Iter(this->m_pRoot, order);
			}

			// ������еĽڵ㣬��ǰ����������Լ�������µĽڵ�
			virtual void Clear()
			{
				this->m_NodeCount = 0;
				CRBNode<K, V> *pTemp = NullNode;

				CTreeIter<K, V> iter = this->Iter(EIterOrder::PostOrder);
				for (iter.Start(); iter.Keepup(); iter.Step())
				{
					if (pTemp != NullNode)
					{
						pTemp->m_pParent = NULL;
						pTemp->m_pLeft = NULL;
						pTemp->m_pRight = NULL;
						delete pTemp;
						pTemp = NULL;
					}

					CRBNode<K, V> *pNode = (CRBNode<K, V>*)iter.GetCurrentNode();
					pTemp = pNode;
				}

				if (pTemp != NullNode)
				{
					pTemp->m_pParent = NULL;
					pTemp->m_pLeft = NULL;
					pTemp->m_pRight = NULL;
					delete pTemp;
					pTemp = NULL;
				}

				this->m_pRoot = NullNode;
				this->ResetNullNode(m_pRoot);
			}

			// ���ٵ�ǰ������󣬲�����ִ����ӻ�ɾ���ڵ����
			virtual void Destroy()
			{
				if (this->m_pRoot == NULL)
					return;

				this->Clear();

				// Clear() ֮��m_pRoot �Ѿ�ָ�� NullNode���ʼ򵥵����� m_pRoot Ϊ NULL ����
				this->m_pRoot = NULL;

				if (NullNode != NULL)
				{
					NullNode->m_pParent = NULL;
					NullNode->m_pLeft = NULL;
					NullNode->m_pRight = NULL;
					delete NullNode;
					NullNode = NULL;
				}
			}

		protected:
			//����key  
			CRBNode<K, V>* FindNode(K key) const
			{
				if (m_pRoot == NullNode)
					return NullNode;

				CRBNode<K, V> *pNode = m_pRoot;
				int flag = 0;
				while (pNode != NullNode)
				{
					flag = m_pComparor(key, pNode->m_Key);
					if (flag < 0)     
					{  
						pNode  = pNode->m_pLeft;  //�ȵ�ǰ��С������  
					}  
					else if (flag > 0)  
					{  
						pNode = pNode->m_pRight;  //�ȵ�ǰ�Ĵ�����  
					}  
					else  
					{  
						break;  
					}  
				}  
				return pNode; 
			}

			void Initial()
			{
				this->NullNode = new CRBNode<K, V>();
				this->NullNode->m_Color = BlackGreen;

				this->m_pRoot = NullNode;
				this->ResetNullNode(m_pRoot);
				
				this->m_NodeCount = 0;
			}

			void ResetNullNode(CRBNode<K, V> *pRoot)
			{
				// NullNode �ĸ��ڵ㡢�����ӽڵ���Զָ����ڵ㡣���ô������ж�һ���ڵ��Ƿ�Ϊ NullNode �ڵ�
				NullNode->m_pParent = pRoot;
				NullNode->m_pLeft = pRoot;
				NullNode->m_pRight = pRoot;
			}

			CTreeIter<K, V> Iter(CRBNode<K, V> *pStartNode, EIterOrder::Enums order) const
			{
				CTreeIter<K, V> iter(pStartNode, order);
				return iter;
			}

			virtual CRBNode<K, V>* InsertNode(const K &key, const V &val)
			{  
				CRBNode<K, V> *newParent = NullNode;
				CRBNode<K, V> *pNode = m_pRoot;
				int flag = -1;
				while (pNode != NullNode)
				{  
					newParent = pNode;
					flag = m_pComparor(key, pNode->m_Key);
					if (flag < 0)
					{
						pNode = pNode->m_pLeft;  
					}
					else if(flag > 0)
					{  
						pNode = pNode->m_pRight; 
					}
					else  
					{
						pNode->m_Value = val;
						return NullNode;  
					}  
				}
				++this->m_NodeCount;
				CRBNode<K, V> *newNode = new CRBNode<K, V>();
				newNode->m_Key = key;
				newNode->m_Value = val;
				newNode->m_Color = Red;  
				newNode->m_pRight = NullNode;
				newNode->m_pLeft = NullNode;
				if (newParent == NullNode) //����������һ�ſ���
				{  
					m_pRoot = newNode;
					m_pRoot->m_pParent = NullNode;
					this->ResetNullNode(m_pRoot);
				}  
				else  
				{  
					if (flag < 0)
						newParent->m_pLeft = newNode;  
					else
						newParent->m_pRight = newNode;

					newNode->m_pParent = newParent;  
				}  
				InsertFixUp(newNode);    //����InsertFixUp�޸����������
				return newNode;
			}

			virtual void InsertFixUp(CRBNode<K, V> *pNode)  
			{
				while (pNode != NullNode && pNode != m_pRoot && pNode->m_pParent->m_Color == Red)
				{  
					if (pNode->m_pParent->IsLeftChild() == true)
					{  
						CRBNode<K, V> *uncle = pNode->m_pParent->m_pParent->m_pRight;  
						if (uncle != NullNode && uncle->m_Color == Red)  
						{ 
							//�������1��pNode�������Ǻ�ɫ
							pNode->m_pParent->m_Color = Black;
							uncle->m_Color = Black;
							pNode->m_pParent->m_pParent->m_Color = Red;
							pNode = pNode->m_pParent->m_pParent;
						}  
						else
						{  
							if (pNode->IsRightChild() == true)
							{  
								//�������2��pNode�������Ǻ�ɫ����pNode������ڵ㲻���ڣ���pNode���丸�ڵ���ҽڵ�
								pNode = pNode->m_pParent;
								RotateLeft(pNode);
							}

							//�������3��pNode�������Ǻ�ɫ��pNode������ڵ㲻���ڣ���pNode���丸�ڵ����ڵ�
							pNode->m_pParent->m_Color = Black;  
							pNode->m_pParent->m_pParent->m_Color = Red;  
							RotateRight(pNode->m_pParent->m_pParent);  
						}  
					}  
					else 
					{  
						CRBNode<K, V> *uncle = pNode->m_pParent->m_pParent->m_pLeft;  
						if (uncle != NullNode && uncle->m_Color == Red)  
						{  
							pNode->m_pParent->m_Color = Black;  
							uncle->m_Color = Black;  
							uncle->m_pParent->m_Color = Red;  
							pNode = pNode->m_pParent->m_pParent;  
						}  
						else
						{  
							if (pNode->IsLeftChild() == true)
							{
								pNode = pNode->m_pParent;
								RotateRight(pNode);     //������������ȣ�������Ϊ����  
							}

							pNode->m_pParent->m_Color = Black;  
							pNode->m_pParent->m_pParent->m_Color = Red;  
							RotateLeft(pNode->m_pParent->m_pParent);   //������Ϊ����
						}  
					}  
				}  
				m_pRoot->m_Color = Black;
			}
		};  
	} // End namespace Collection
}// End namespace Ext
