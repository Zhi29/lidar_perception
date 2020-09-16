import cv2
import os

img_root = '/home/lizhi/sample_test_result/img_results_corrected_new_data/PLEVZ744_recording_recording_rviz_20200814-192444_20200814-192549_0/fisheye_B_lidar'  # 这里写你的文件夹路径，比如：/home/youname/data/img/,注意最后一个文件夹要有斜杠
Files = os.listdir(img_root)
Files.sort()
fps = 20.0  # 保存视频的FPS，可以适当调整
size = (1280, 960)
# 可以用(*'DIVX')或(*'X264'),如果都不行先装ffmpeg: sudo apt-get install ffmpeg
fourcc = cv2.VideoWriter_fourcc(*'XVID')
videoWriter = cv2.VideoWriter('PLEVZ744-192549_0_fisheye_B.avi', fourcc, fps, size)  # 最后一个是保存图片的尺寸

# for(i=1;i<471;++i)
for i in range(len(Files)):
    if '.jpg' in Files[i]:
        frame = cv2.imread(img_root + '/' + Files[i])
        videoWriter.write(frame)
videoWriter.release()