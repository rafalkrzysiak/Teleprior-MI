source='stemfest_film.mp4';
vidobj=VideoReader(source);
frames=vidobj.NumFrames;
for f=1:frames
    thisframe=read(vidobj,f);
    figure(1);imagesc(thisframe);
    thisfile = sprintf('frames/%04d.jpg', f);
    imwrite(thisframe, thisfile);
end