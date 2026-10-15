std_msgs/Header header

# Coarse Box: [Mean ESDF, Mask] x 12 x 12 x 12
# Dimensions: 2 channels * 1728 cells = 3456
float32[3456] coarse_box

# Detail Box: [Min ESDF, Mean ESDF, Mask] x 12 x 12 x 12
# Dimensions: 3 channels * 1728 cells = 5184
float32[5184] detail_box