clear all;
close all;
clc;

I_rgb = imread('input_images/vegetables.jpeg');
I = rgb2gray(I_rgb);

sigma = 5;

I_smooth = imgaussfilt(I,sigma);



imwrite(I_smooth,"output_images/image_gaussian_smoothing.jpeg");