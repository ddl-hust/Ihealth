### how to build ihealth from scratch
1. git clone git@github.com:ddl-hust/Ihealth.git

2. 为了避免将很多第三方库，提交到github ,我们在gitignore 添加了Boost_1_67_0，eigen,TBB,opencv(later will remove)
因此我们需要将这些库人为添加到ihealth文件夹，这个比较鸡肋[To-Do]

3. **cmake** include third library

### Issue 
1. 配置cmake build 提示缺少XXX.dll 
2. param 文件没有导入 导致 XXX错误
3. 没导入ihealth.sql 导致在界面里面添加患者，信息不能保存