
    function surf_del( n )

        % create point set %
        p = rand(n,2);

        % point set deformation %
        p(:,1) = p(:,1) * 5;
        p(:,2) = p(:,2) * 5;
        a = rand * 2 * pi; r = [ cos(a), sin(a); -sin(a), cos(a) ];
        p = ( r * p' )';

        % compute delaunay triangulation %
        t = delaunay( p(:,1), p(:,2) );

        % plot %
        figure;
        hold on;
        plot( p(:,1), p(:,2), '.r' );
        triplot( t, p(:,1), p(:,2), 'k' );
        daspect([1 1 1]);

        % compute and display surface value %
        printf( 'surface delaunay : %f\n', delaunay_surf( t, p(:,1), p(:,2) ) );

        % compute and display surface value %
        printf( 'surface cov. : %f\n', surface( p(:,1), p(:,2) ) );

    end

    function s = delaunay_surf( t, x, y )

        % initialse value %
        s = 0.0;

        % parsing triangle %
        for i = 1 : size( t, 1 )

            % update surface %
            s = s + abs( 0.5 * det( [ x(t(i,1)), y(t(i,1)), 1; x(t(i,2)), y(t(i,2)), 1; x(t(i,3)), y(t(i,3)), 1 ] ) );

        end

    end

    function s = surface( x, y )

        % compute centroid %
        c = [ sum(x), sum(y) ] / length( x );

        % compose random variable vector - 2D %
        p = [ x, y ];

        % compute covariance matrix %
        h = ( 1 / ( length( x ) - 1 ) ) * ( ( p - c )' * ( p - c ) );

        % svd decomposition for eigenvectors and eigenvalues %
        [ u s v ] = svd( h );

        % extract non-correlated variances %
        s = sqrt( s );

        % extract eigenvectors %
        v1 = [u(1,1), u(2,1)];
        v2 = [u(1,2), u(2,2)];

        % apply scale factor with correction %
        v1 = s(1,1) * v1 * 1.5;
        v2 = s(2,2) * v2 * 1.5;

        % display eigenvectors %
        plot( c(1) + [ 0 v1(1) ], c(2) + [ 0 v1(2) ], '-b', 'linewidth', 2 );
        plot( c(1) + [ 0 v2(1) ], c(2) + [ 0 v2(2) ], '-b', 'linewidth', 2 );

        % compute surface estimation %
        s = s(1,1) * s(2,2) * 4 * 3;

    end
