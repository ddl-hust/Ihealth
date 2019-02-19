// 定义手类，用手抓鸡蛋
function Hand(x, y, imagesrc) {
	this.x = x
	this.y = y
	this.imagesrc = imagesrc
	this.node = null
	this.range_x = maindiv.clientWidth
	this.range_y = maindiv.clientHeight
	
	this.init = function() {
		this.node=document.createElement("img");
		this.node.style.position='absolute';
		this.node.style.left=this.x+'px';
		this.node.style.top=this.y+'px';
		this.node.src=this.imagesrc;
	}
	// 在new的时候就直接进行init
	this.init()
	
	// 在new的时候就直接向maindiv中加入这个node
	maindiv.appendChild(this.node)
	
	this.move_to = function(x, y) {
		this.x = x
		this.y = y
		// 注意飞机不能超出界面
		var left = 0
		if (this.x < left) {
			this.x = left;
		}
		
		var right = this.range_x - this.node.clientWidth;
		if (this.x > right) {
			this.x = right		
		}
		
		var top = 0
		if (this.y < top) {
			this.y = top
		}
		
		var down = this.range_y - this.node.clientHeight
		if (this.y > down) {
			this.y = down
		}
		
		this.node.style.left =	this.x + 'px'
		this.node.style.top = this.y + 'px'	
	}
	
	// 使手的图片变成鸡蛋，用于拿到了鸡蛋之后
	this.become_egg = function() {
		this.imagesrc = "image/egg.png"
		this.node.src = this.imagesrc
	}
	
	// 把手的图片还原，用于鸡蛋被煎了之后
	this.become_hand = function() {
		this.imagesrc = "image/hand.png"
		this.node.src = this.imagesrc
	}
}

