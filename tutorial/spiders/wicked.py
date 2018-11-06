import scrapy 
import pandas as pd 


class WickedSpider(scrapy.Spider):
	name='Aragog'
	start_urls = ["https://www.jumia.com.ng/watches/", "https://www.jumia.com.ng/phones-tablets/", 
	"https://www.jumia.com.ng/computing/", "https://www.jumia.com.ng/electronics/", "https://www.jumia.com.ng/home-office/", 
	"https://www.jumia.com.ng/category-fashion-by-jumia/",
	"https://www.jumia.com.ng/health-beauty/",
	"https://www.jumia.com.ng/video-games/",
	"https://www.jumia.com.ng/groceries/",
	"https://www.jumia.com.ng/baby-products/",
	"https://www.jumia.com.ng/toys-games/",
	"https://www.jumia.com.ng/sporting-goods/",
	]

	def parse(self, response):
		filename = response.url.split("/")[-2] +'.xlsx'	
		
		discount = response.xpath('//*[@class="sale-flag-percent"]/text()').extract()
		item_name = response.xpath('//*[@class="name"]/text()').extract()
		
		x = len(discount)
		y = 0
		item_discount_list = []
		while y<x:
			xy =[]
			threshold = int(discount[y].split('%')[0])
			if threshold < -50:	
				xy.append(int(discount[y].split('%')[0]))
				xy.append(item_name[y])
				item_discount_list.append(xy)
			
			else:
				pass
			y += 1

		df = pd.DataFrame(item_discount_list, columns=['Discount', 'Item'])
		writer = pd.ExcelWriter(filename, engine='xlsxwriter')
		df.to_excel(writer, sheet_name='Sheet1')

			
