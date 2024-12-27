% function to normalize images
srcPath = 'D:\Workstation\CVPR2010\Supplementary\image\foliage\';
%srcPath = 'D:\Workstation\CVPR2010\Supplementary\index\foliage\';
%srcPath = 'D:\Workstation\CVPR2010\Supplementary\selected\foliage\';

imagelist = dir(fullfile(srcPath,'*.jpg'));

figure;
for i = 1:length(imagelist)
    im = imread(fullfile(srcPath,imagelist(i).name));
    im = im2double(im);
    Mean = mean(im(:));
    Std  = std(im(:));
    Im = (im-Mean)/Std*0.28+0.6;
    Im = im*0.25+Im*0.75;
    imshow(Im);drawnow;
    imwrite(Im,fullfile(srcPath,imagelist(i).name),'qualit',100);
end