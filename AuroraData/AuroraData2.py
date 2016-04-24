from auromat.mapping.spacecraft import SpacecraftMappingProvider
from auromat.resample import resample
from auromat.draw import saveFig
import auromat.draw as draw
import numpy as np
import cv2
import base64
import math
import matplotlib.pyplot as plt

for i in range(64757, 64758):

	dcfolder = '/home/lianqiang/data/iss_imgs_dc'
	wcsfolder = '/home/lianqiang/data/iss_wcs'

	print("Calculating...")
	provider = SpacecraftMappingProvider(dcfolder, wcsfolder)
	number = str(i)
	mapping = provider.getById(number) # id = any unique part of the filename
	resampled = resample(mapping, arcsecPerPx=100)
	masked = resampled.maskedByElevation(10)
	# h = mapping.img.shape[0]
	# w = mapping.img.shape[1]

	# img = list(np.array(masked.img).flatten())
	# lats = list(np.array(masked.lats).flatten())
	# lons = list(np.array(masked.lons).flatten())

	img = masked.img
	lats = masked.lats
	lons = masked.lons

	# plt.scatter(lats, lons, np.array(masked.img).flatten())
	# plt.show()
	# for i in range(0, len(lats)):
	# 	#print(lats[i])
	# 	if lats[i] != lats[i] or lons[i] != lons[i]:
	# 		del lats[i]
	# 		del lons[i]
	# 		del img[i:i+3]
	# 	# else:
	# 	# 	print(lats[i])

	# print(len(img))
	# print(len(lats))
	# print(len(lons))

	# f = open('/home/lianqiang/data/img'+number, 'a', 1000)
	# f.write(str(img))
	# f.close()
	# f = open('/home/lianqiang/data/lats'+number, 'a', 1000)
	# f.write(str(lats))
	# f.close()
	# f = open('/home/lianqiang/data/lons'+number, 'a', 1000)
	# f.write(str(lons))
	# f.close()

	saveFig('/home/lianqiang/data/map_test.png', draw.drawPlot(masked))
	#saveFig('/home/lianqiang/data/map.png', draw.drawStereographic(masked, lineDelta=2, width=1600, height=1000,))
	#saveFig('/home/lianqiang/data/astrometry.jpg', draw.drawReferenceStars(mapping, scale=2))