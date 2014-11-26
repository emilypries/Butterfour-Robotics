function CameraTest()
    
    %Will get an initial snapshot of the camera and will get a single input
    %from the user which will decide the target color
    str_ip = 'http://192.168.0.102/snapshot.cgi?user=admin&pwd=&resolution=16&rate=0';
    img = imread(str_ip);
    fh = figure;, imshow(img);
    [x, y] = ginput(1); 
    delete(fh);

    %Gets target color, CURRENTLY RGB 
    img_hsv = rgb2hsv(img);
    trgt = img_hsv(floor(y), floor(x), :)
    range = 0.02;
    
    %Endless loop to test camera, quit with ctrl c :)
    while true
        %Reads current image
        img = rgb2hsv(imread(str_ip));

        %Sets up and constructs mask
        
        in_mask_L = img(:, :, 1) > (trgt - range);
        in_mask_H = img(:, :, 1) < (trgt + range);
        in_mask = (in_mask_L == in_mask_H);
        
        %Performs 15 dilations and erosions. Fills in holes in blobs that
        %are meant to connect together
        k = 15;
        processed_img = bwmorph(in_mask, 'dilate', k);
        processed_img = bwmorph(processed_img, 'erode', k);

        %Performs 15 erosions and 15 dilations. Removes some random noise in
        %the image
        k = 15;
        processed_img = bwmorph(processed_img, 'erode', k);
        processed_img = bwmorph(processed_img, 'dilate', k);
        
        in_mask = processed_img;
        
        %Creates a labeled image. Each label (int value) refers to an
        %object in the mask where 1's are connected
        lbl_mask = bwlabel(in_mask);
        
        %Finds the blob with the biggest area. This will be our target.
        A = 0;
        label = 1;
        mask = in_mask;
        while true
            nLabel = lbl_mask == label;
            if(~any(nLabel(:)))
                break;
            end
            A_curr = sum(sum(nLabel));
            if(A_curr > A)
                A = A_curr;
                mask = nLabel;
            end
            label = label + 1;
        end
            
        %Calculates the centroid of the main target
        A = sum(sum(mask));
        x = 0;
        y = 0;
        [m_mask, n_mask] = size(mask);
        
        for i = 1:m_mask
            for j = 1:n_mask
                if(mask(i, j))
                    x = x + j;
                    y = y + i;
                end
            end
        end

        %Centroids
        x_cen = floor(x/A);
        y_cen = floor(y/A);

        fh = figure;
        imshow(mask);
        hold on;
        %Draws square over centroid.
        plot([x_cen-10;x_cen+10], [y_cen+10;y_cen+10],...
            'r-', 'MarkerFaceColor', [1 0 0]);
        plot([x_cen-10;x_cen+10], [y_cen-10;y_cen-10],...
            'r-', 'MarkerFaceColor', [1 0 0]);
        plot([x_cen+10;x_cen+10], [y_cen-10;y_cen+10],...
            'r-', 'MarkerFaceColor', [1 0 0]);
        plot([x_cen-10;x_cen-10], [y_cen-10;y_cen+10],...
            'r-', 'MarkerFaceColor', [1 0 0]);
        pause(0.05);
        delete(fh);
    end

end