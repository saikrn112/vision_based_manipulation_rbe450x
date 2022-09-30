clear all;
close all;
clc;

I_rgb = imread('input_images/vegetables.jpeg');
I = rgb2gray(I_rgb);


imwrite(I,"output_images/veg_gray.jpeg");