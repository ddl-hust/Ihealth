// 找到在html中定义的div
var startdiv=document.getElementById('startdiv')
var maindiv=document.getElementById('maindiv')
var score=document.getElementById('score')
var totalscore=document.getElementById('totalscore')
var enddiv=document.getElementById('enddiv')

var log = console.log


// 加入一个抹布
window.m_rag = null

// 加入一个窗户
window.m_window = null

// 得分
var scores = 0

var tid

// 定义游戏开始的函数，这时候应该显示maindiv，隐藏startdiv
function begin(){
	startdiv.style.display='none'
	maindiv.style.display='block'
	
	// 先添加窗户，再添加抹布，这样抹布才会画在窗户上面
	m_window = new Window(200, 100)
	// 最右是710
	m_rag = new Rag(0, 596, "image/rag.png", 2)	
	tid = setInterval(main_loop, 200)
	return 'start'
}

var main_loop = function() {
	var rag = window.m_rag
	var win = window.m_window
	
	if (clean_window(rag, win)) {
		scores += win.current_score()	
		win.clean_once()
		score.innerHTML = scores
	}
}

// 定义擦窗户成功的判定函数，现在以抹布从窗户外进入窗户内，然后又出窗户定义为一次擦窗户
var clean_window_state = 0
var clean_window = function(rag, win) {
	switch(clean_window_state) {
		case 0:
			// 初始状态下，这时候只有抹布在窗户内部才能使得状态是1
			if (rag_in_window(rag, win)) {
				clean_window_state = 1
			}
			return false;
		case 1:
			// 这时候抹布在窗户内部，只有抹布和窗户相离才能使得状态是0
			if (rag_disjoint_with_window(rag, win)) {
				clean_window_state = 0
				return true
			}
		default:
			return false
	}
}

// 抹布和窗户相离
var rag_disjoint_with_window = function(rag, win) {
	var rag_w = rag.node.clientWidth // 抹布的宽度
	var rag_h = rag.node.clientHeight // 抹布的高度
	var rag_l = rag.x // 抹布的最左边
	var rag_t = rag.y // 抹布的最上边
	
	var win_w = win.node.clientWidth // 窗户的宽度
	var win_h = win.node.clientHeight // 窗户的高度
	var win_l = win.x // 窗户的最左边
	var win_t = win.y // 窗户的最右边
	
	if (rag_l > win_l + win_w || rag_l + rag_w < win_l || 
		rag_t + rag_h < win_t || rag_t > win_t + win_h) {
			return true
	}
	
	return false
}

// 抹布在窗户内部
var rag_in_window = function(rag, win) {
	var rag_w = rag.node.clientWidth // 抹布的宽度
	var rag_h = rag.node.clientHeight // 抹布的高度
	var rag_l = rag.x // 抹布的最左边
	var rag_t = rag.y // 抹布的最上边
	
	var win_w = win.node.clientWidth // 窗户的宽度
	var win_h = win.node.clientHeight // 窗户的高度
	var win_l = win.x // 窗户的最左边
	var win_t = win.y // 窗户的最右边
	
	//log(rag_w, rag_h, rag_l, rag_t, win_w, win_h, win_l, win_t)

	if (rag_l > win_l && rag_l + rag_w < win_l + win_w &&
		rag_t > win_t && rag_t + rag_h < win_t + win_h) {
		return true
	}
	
	return false
}

// 和C++端对应的接口，用来区分不同的游戏
function getGameType() {
	return '擦窗户'	
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

// 控制抹布运动到x, y
var selfmove = function(x, y) {
	rag = window.m_rag
	rag.move_to(x, y)
}






