// ��ͬ�׶δ�����ͼƬ��Դ
var window_srcs = 
[
	"image/window_dog_step4.png",
	"image/window_dog_step3.png",
	"image/window_dog_step2.png",
	"image/window_dog_step1.png",
	"image/window_dog.png",
	"image/window_snowman_step4.png",
	"image/window_snowman_step3.png",
	"image/window_snowman_step2.png",
	"image/window_snowman_step1.png",
	"image/window_snowman.png",
	"image/window_forest_step4.png",
	"image/window_forest_step3.png",
	"image/window_forest_step2.png",
	"image/window_forest_step1.png",
	"image/window_forest.png",
	"image/window_sea_step4.png",
	"image/window_sea_step3.png",
	"image/window_sea_step2.png",
	"image/window_sea_step1.png",
	"image/window_sea.png",
]

// ��ͬ�׶δ��������ļӷ�
var clean_scores = 
[
	100, 100, 100, 100, 400,
	100, 100, 100, 100, 400,
	100, 100, 100, 100, 400,
	100, 100, 100, 100, 400,
]

// ���崰����
function Window(x, y) {
	this.x = x
	this.y = y
	this.imagesrcs = window_srcs
	this.image_index = 0
	this.node = null
	
	this.init = function() {
		this.node=document.createElement("img");
		this.node.style.position='absolute';
		this.node.style.left=this.x+'px';
		this.node.style.top=this.y+'px';
		this.node.src=this.imagesrcs[this.image_index];
	}
	// ��new��ʱ���ֱ�ӽ���init
	this.init()
	
	// ��new��ʱ���ֱ����maindiv�м������node
	maindiv.appendChild(this.node)
	
	// ���Ĵ�����ͼƬ,����һ�δ���֮��Ҫ��һ��ͼƬ
	this.clean_once = function() {
		if (this.image_index + 1 >= this.imagesrcs.length) {
			this.image_index = 0	
		} else {
			this.image_index++	
		}
		
		this.node.src = this.imagesrcs[this.image_index]
	}
	
	this.current_score = function() {
		return clean_scores[this.image_index];		
	}
}