// 定义抹布类
function Rag(x, y, imagesrc, speed) {
	this.x = x
	this.y = y
	this.imagesrc = imagesrc
	this.speed = speed
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
}

