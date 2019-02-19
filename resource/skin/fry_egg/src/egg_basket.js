// 定义鸡蛋篮子类
function EggBasket(x, y, imagesrc) {
	this.x = x
	this.y = y
	this.imagesrc = imagesrc
	this.node = null
	
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
}