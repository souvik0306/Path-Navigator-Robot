clc 
clear all 
% tform = trvec2tform(trvec) converts the Cartesian representation of a translation vector, trvec, to the corresponding homogeneous transformation, tform. When using the
% transformation matrix, premultiply it with the coordinates to be transformed (as opposed to postmultiplying). 
trvec = [0.5 6 100]
tform = trvec2tform(trvec)
