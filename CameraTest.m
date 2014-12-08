function CameraTest()
    
    %Will get an initial snapshot of the camera and will get a single input
    %from the user which will decide the target color
    str_ip = 'http://192.168.0.100/snapshot.cgi?user=admin&pwd=&resolution=16&rate=0';
    img = imread(str_ip);
    fh = figure; imshow(img);
    h = imrect;
    rect = round(h.getPosition);
    delete(fh);
    
    %Extract values from rect
    width = rect(3);
    height = rect(4);
    x = rect(1) + round(width/2);
    y = rect(2) + round(height/2);

    %Gets target color, CURRENTLY HSV
    img_hsv = rgb2hsv(img);
    trgt_window = img_hsv(rect(2):rect(2)+height,rect(1):rect(1)+width,:);
    trgt = mean(mean(trgt_window))
    hr = 0.05;
    sr = 0.16;
    vr = 0.15;
    
    fh = figure;
    
    %Endless loop to test camera, quit with ctrl c :)
    while true
        
        %Reads current image
        img = rgb2hsv(imread(str_ip));
        imwrite(img, 'hsv.png');
        rgb = hsv2rgb(img);
        subplot(2, 3, 4);
        imshow(rgb); title('RGB');
        subplot(2, 3, 6);
        imshow(img); title('HSV');
        %Sets up and constructs mask
        h_mask_L = img(:, :, 1) > (trgt(1) - hr);
        h_mask_H = img(:, :, 1) < (trgt(1) + hr);
        s_mask_L = img(:, :, 2) > (trgt(2) - sr);
        s_mask_H = img(:, :, 2) < (trgt(2) + sr);
        v_mask_L = img(:, :, 3) > (trgt(3) - vr);
        v_mask_H = img(:, :, 3) < (trgt(3) + vr);
        avg_mask = (h_mask_L == h_mask_H) &...
                   (s_mask_L == s_mask_H);% &...
                   %(v_mask_L == v_mask_H);
        subplot(2, 3, 1);
        imshow((h_mask_L == h_mask_H)); title('Hue');
        subplot(2, 3, 2);
        imshow((s_mask_L == s_mask_H)); title('Saturation');
        subplot(2, 3, 3);
        imshow((v_mask_L == v_mask_H)); title('Value');
        
    
        %Performs 15 dilations and erosions. Fills in holes in blobs that
        %are meant to connect together
        k = 10;
        processed_img = bwmorph(avg_mask, 'dilate', k);
        processed_img = bwmorph(processed_img, 'erode', k);

        %Performs 15 erosions and 15 dilations. Removes some random noise in
        %the image
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

        subplot(2,3,5);
        imshow(mask); title('Mask');
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
        hold off;
        
    end

end