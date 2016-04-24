from auromat.solving.eol import downloadImageSequence

imgfolder = '/home/lianqiang/data/iss_imgs'
#metadata, failures = downloadImageSequence(imgfolder, 'ISS040', 64750, 64770, 'jpg')

from auromat.solving.eol import correctLensDistortion
dcfolder = '/home/lianqiang/data/iss_imgs_dc'
#correctLensDistortion(imgfolder, dcfolder)
print("Store Distortion Corrected Images in /data/iss_imgs_dc")

from auromat.solving.spacecraft import solveSequence

wcsfolder = '/home/lianqiang/data/iss_wcs'
stuser, stpass = 'space-track-user', 'space-track-pass'
noradid = 25544 # NORAD ID of ISS
tlefolder = '/home/lianqiang/data/tles' # location where orbit data from space-track.org is cached
s = list(solveSequence(dcfolder, wcsfolder, tlefolder, stuser, stpass, noradid))
print("Store Solved Images in /data/iss_wcs")
