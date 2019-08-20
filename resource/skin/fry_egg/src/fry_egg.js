// 找到在html中定义的div
var startdiv=document.getElementById('startdiv')
var maindiv=document.getElementById('maindiv')
var score=document.getElementById('score')
var totalscore=document.getElementById('totalscore')
var enddiv=document.getElementById('enddiv')

var log = console.log


// 加入一个盘子
window.m_plate = null

// 加入一个平底锅
window.m_pan = null

// 加入一个鸡蛋篮子
window.m_egg_basket = null

// 加入一个只手
window.m_hand = null

// 做好了的鸡蛋
window.m_cooked_egg = null

// 得分
var scores = 0

var tid

// 定义游戏开始的函数，这时候应该显示maindiv，隐藏startdiv
function begin(){
	startdiv.style.display='none'
	maindiv.style.display='block'
	
	// 添加盘子，平底锅和鸡蛋篮子等不动的物件
	m_plate = new Plate(0, 100, "image/plate.png")
	m_pan = new Pan(530, 80, "image/pan.png")
	m_egg_basket = new EggBasket(0, 477, "image/egg_basket.png")
	
	// 添加一个能动的手
	m_hand = new Hand(0, 581, "image/hand.png")
	
	// 添加做好的鸡蛋
	m_cooked_egg = new CookedEgg(75, 81, "image/fried_egg.png")
	
	tid = setInterval(fry_egg, 200)
}


// 定义擦窗户成功的判定函数，现在以抹布从窗户外进入窗户内，然后又出窗户定义为一次擦窗户
window.fry_egg_state = 0
var fry_egg = function() {
	var hand = window.m_hand
	var cooked_egg = window.m_cooked_egg
	var egg_basket = window.m_egg_basket
	var pan = window.m_pan
	switch(window.fry_egg_state) {
		case 0:
			// 初始状态下，这时候手在鸡蛋筐内部，只有手在鸡蛋筐外面去，状态才变为1
			if (A_disjoint_with_B(hand, egg_basket)) {
				hand.become_egg()
				fry_egg_state = 1	
			}
			return false
		case 1:
			// 这时候手变成了鸡蛋，只有手到了平底锅内部，这时候状态才变成2
			if (A_in_B(hand, pan)) {
				// 手变成手型
				hand.become_hand()
				
				// 做好的鸡蛋在平底锅里面
				cooked_egg.move_to(580,140)
				cooked_egg.append_to_maindiv()
				
				fry_egg_state = 2
			}
			
			return false
		case 2:
			// 只有手离开了平底锅，这时候状态才变成3
			if (A_disjoint_with_B(hand, pan)) {
				// 将鸡蛋放到盘子里面
				cooked_egg.move_to(75, 181)
				
				// 增加分数
				scores += 100
				score.innerHTML = scores
				
				fry_egg_state = 3
			}
			return false
		case 3:
			// 手已经离开了平底锅，现在要等手进入鸡蛋筐的时候，再次把状态调整为0
			if (A_in_B(hand, egg_basket)) {
				// 把盘子里面的鸡蛋移除
				cooked_egg.remove_from_maindiv()
				
				fry_egg_state = 0
				return true	
			}
			
			return false
		default:
			return false
	}
}

// A和B相离函数
var A_disjoint_with_B = function(A, B) {
	var rag_w = A.node.clientWidth // 抹布的宽度
	var rag_h = A.node.clientHeight // 抹布的高度
	var rag_l = A.x // 抹布的最左边
	var rag_t = A.y // 抹布的最上边
	
	var win_w = B.node.clientWidth // 窗户的宽度
	var win_h = B.node.clientHeight // 窗户的高度
	var win_l = B.x // 窗户的最左边
	var win_t = B.y // 窗户的最右边
	
	if (rag_l > win_l + win_w || rag_l + rag_w < win_l || 
		rag_t + rag_h < win_t || rag_t > win_t + win_h) {
			return true
	}
	
	return false
}

// A在B内部
var A_in_B = function(A, B) {
	var rag_w = A.node.clientWidth // 抹布的宽度
	var rag_h = A.node.clientHeight // 抹布的高度
	var rag_l = A.x // 抹布的最左边
	var rag_t = A.y // 抹布的最上边
	
	var win_w = B.node.clientWidth // 窗户的宽度
	var win_h = B.node.clientHeight // 窗户的高度
	var win_l = B.x // 窗户的最左边
	var win_t = B.y // 窗户的最右边

	if (rag_l >= win_l && rag_l + rag_w <= win_l + win_w &&
		rag_t >= win_t && rag_t + rag_h <= win_t + win_h) {
		return true
	}
	
	return false
}

// 和C++端对应的接口，用来区分不同的游戏
function getGameType() {
	return '煎鸡蛋'	
}

function getWidth() {
	var x = document.documentElement.clientWidth;//selfplane.x*2;
	return x
};

function getHeight() {
	return document.documentElement.clientHeight;// selfplane.y
};

function getScore() {
	return scores	
}

// 控制手运动到x, y
var selfmove = function(x, y) {
	hand = window.m_hand
	hand.move_to(x, y)
}


/*
// 设置对键盘的监听事件
document.onkeydown = function(event) {
	var e = event
	var k = e.keyCode
	m_hand= window.m_hand
	m_egg_basket = window.m_egg_basket
	
	var speed = 20
	if (k == 38) {
		// up
		m_hand.move_to(m_hand.x, m_hand.y-speed)
	} else if (k == 40) {
		// down		
		m_hand.move_to(m_hand.x, m_hand.y + speed)
	} else if (k == 37) {
		// left	
		m_hand.move_to(m_hand.x-speed, m_hand.y)
	} else if (k == 39) {
		// right	
		m_hand.move_to(m_hand.x + speed, m_hand.y)
	}
}
*/



