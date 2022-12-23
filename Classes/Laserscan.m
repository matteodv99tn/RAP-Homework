classdef Laserscan 

    properties 
        polar;                  % for each row there's a pair [radius, theta]
        cartesian;              % for each row there's a pair [x coords, y coords]
        features = [];
    end % properties



    methods 

        function obj = Laserscan(measures, angles)
            
            obj.polar       = [ to_column_vector(measures), to_column_vector(angles) ];
            obj.cartesian   = [ cos(obj.polar(:, 2)) .* obj.polar(:, 1), ...
                                sin(obj.polar(:, 2)) .* obj.polar(:, 1)  ];

            function out = to_column_vector(in)
                if isrow(in)
                    out = in';
                else
                    out = in;
                end
            end

        end


        function plot(obj)

            plot(obj.cartesian(:, 1), obj.cartesian(:, 2), '.k', ...
                'DisplayName', 'point cloud');

        end

        
        function feat = extract_feature(obj)

            segments = obj.seeding();
            figure, clf, hold on;
            plot(obj);
            for k = 1:size(segments, 1)

                p1 = obj.cartesian(segments(k, 1), :);
                p2 = obj.cartesian(segments(k, 2), :);

                plot([p1(1), p2(1)], [p1(2), p2(2)], '-r', 'LineWidth', 2);

            end 

            segments = obj.segment_reduction(segments);   

            for k = 1:size(segments, 1)

                p1 = obj.cartesian(segments(k, 1), :);
                p2 = obj.cartesian(segments(k, 2), :);

                plot([p1(1), p2(1)], [p1(2), p2(2)], 'o-.g', 'LineWidth', 2);

            end

            feat = segments;

        end

        function seeds = seeding(obj)

            seeds = [];
            N_min = 8;
            N_back = 2;
            N   = size(obj.cartesian, 1);

            i = 1;
            j = i + N_min;

            while j < N

                line = obj.fit_line(i, j);
                if obj.check_line_correctness(line, i, j)
                    seeds(end+1, :) = [i, j, line];
                    i = j - N_back;
                else
                    i = i + 1;
                end

                j = i + N_min;

            end


        end

        function res_seeds = join_seeds(obj, seed_a, seed_b)


            v1 = seed_a(3:4);
            v2 = seed_b(3:4);

            res_seeds = [seed_a; seed_b];

            if (seed_b(1) > seed_a(2)) 
                return;
            end

            confr = (v1 * v2') / (norm(v1) * norm(v2));
            if abs(confr) > cos(5* pi / 180) 

                line = obj.fit_line(seed_a(1), seed_b(2));

                if obj.check_line_correctness(line, seed_a(1), seed_b(2))
                    res_seeds = [seed_a(1), seed_b(2), line];
                end
            end

        end

        function features_new = segment_reduction(obj, segments)

            features_old = segments;
            features_new = [];

            for tt = 1:20
                
                disp(['Iter ' num2str(tt), ' - Dim ', num2str(size(features_old, 1))]);
                
                k = 1;
                features_new  = [];

                while k < (size(features_old, 1) - 1)
                    
                    reduced_feature = obj.join_seeds(features_old(k,:), features_old(k+1,:));

                    if size(reduced_feature, 1) == 1
                        features_new(end+1, :) = reduced_feature;
                        k = k + 2;
                    else
                        features_new(end+1, :) = reduced_feature(1, :);
                        k = k + 1;
                    end

                    if k == size(features_old, 1) && size(reduced_feature, 1) == 2
                        features_new(end+1, :) = reduced_feature(2, :);
                    end

                end

                features_old = features_new;
            end

        end

        function is_line = check_line_correctness(obj, line, i, j)

            is_line = true;

            delta = 5;
            epsilon = 0.2;
            theta = obj.polar(:,2);

            p1 = obj.cartesian(i, :);
            p2 = obj.cartesian(j, :);
            
            mean_radius = mean(obj.polar([i,j], 1));

            if point_point_distance(p1, p2) > 0.5 * mean_radius 
                is_line = false;
                return;
            end

            for k = i:j

                p_true = obj.cartesian(k, :);
                p_pred = predict_point(line, theta(k));

                % if point_point_distance(p_pred, obj.cartesian(k, :)) > delta
                %     is_line = false;
                %     return
                % end 
                
                if line_point_distance(line, p_true) > epsilon 
                    is_line = false;
                    return
                end
            end

        end


        function coeff = fit_line(obj, i, j)

            X = obj.cartesian(i:j, 1);
            Y = obj.cartesian(i:j, 2);
            I = ones(length(X), 1);

            if std(X) > std(Y)
                A = [I, X];
                b = Y;
                X = pinv(A) * b;
                coeff = [X(2), -1, X(1)];
            else
                A = [I, Y];
                b = X;
                X = pinv(A) * b;
                coeff = [-1, X(2), X(1)];
            end

        end





    end % methods
end % Laserscan class