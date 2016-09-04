package geodesics;

import java.awt.image.BufferedImage;
import java.awt.Color;
import java.io.File;
import java.io.IOException;
import javax.imageio.ImageIO;
import java.lang.Math ; 
import java.util.Comparator;
import java.util.PriorityQueue;


public class Geodesic 
{
	int height ;
	int width ; 
	float metricMap[][] ;
	float distanceMap[][] ;
	float gradientMapX[][] ;
	float gradientMapY[][] ;
	Point start ; 
	Point target ; 
	
	public Geodesic (){} ; 
	
	public Geodesic ( float[][] metricMap , Point start, Point target )
	{
		this.metricMap = metricMap; 
		this.height = metricMap.length ; 
		this.width = metricMap[0].length ; 
		this.start = start ; 
		this.target = target ; 
	}
	
	private Comparator<Point> pointComparator = new Comparator<Point>()
	{
		@Override
		public int compare(Point p1, Point p2) 
		{
			float dist1 = distanceMap[ p1.getY() ][ p1.getX() ] ;
			float dist2 = distanceMap[ p2.getY() ][ p2.getX() ] ;
            return (int) ( dist1 - dist2 );
        }
	};
	
	public void calcDistanceMap ()
	{
		distanceMap = new float [height][width] ; 
		boolean visited[][] = new boolean[height][width] ;
 		
		// Initialize distanceMap
		for ( int y = 0 ; y < height ; y++ )
			for ( int x = 0 ; x < width ; x++ )
					distanceMap[y][x] = 10000000.0f ; 
		
		for ( int y = 0 ; y < height ; y++ )
			for ( int x = 0 ; x < width ; x++ )
					visited[y][x] = false ; 
		
		distanceMap[ start.getY()][ start.getX()] = 0.0f ; 
		  
	 	// Dijkstra's Algorithm
	 	PriorityQueue<Point> Q = new PriorityQueue<Point>(1,pointComparator);
	 	Q.add(start) ;
	 	
	 	while ( !Q.isEmpty() )
	 	{
	 		Point curr = Q.poll()  ;    
	 		int currX = curr.getX() ; 
	 		int currY = curr.getY() ; 
	 		visited[ currY ][ currX ] = true ; 

	 		// Check Neighbors
	 		int dx[] = {-1,1,0,0} ; 
	 		int dy[] = {0,0,1,-1} ;

	 		for ( int i = 0 ; i < 4 ; i++ )
		    {
		    	int newX = currX + dx[ i ] ; 
		    	int newY = currY + dy[ i ] ; 
		    	
		    	if ( newY >= 0 && newY < height &&
		    		 newX >= 0 && newX < width  && 
		    		 !visited[ newY ][ newX ] )
		    	{
		    		// Run Fast Marching Algorithm 
		    		
		    		float minY = Float.MAX_VALUE ; 
		    		if ( newY - 1 >= 0 && newY - 1 < height ) minY = Math.min( minY , distanceMap[ newY - 1 ][ newX ])  ;
		    		if ( newY + 1 >= 0 && newY + 1 < height ) minY = Math.min( minY , distanceMap[ newY + 1 ][ newX ])  ;
		    		
		    		float minX = Float.MAX_VALUE ; 
		    		if ( newX - 1 >= 0 && newX - 1 < width ) minX = Math.min( minX , distanceMap[ newY ][ newX - 1 ])  ;
		    		if ( newX + 1 >= 0 && newX + 1 < width ) minX = Math.min( minX , distanceMap[ newY ][ newX + 1 ])  ;
		    		
			 		float delta = 2.0f*metricMap[ newY ][ newX ]*metricMap[ newY ][ newX ] - ( minX - minY )*( minX - minY ) ;
			 		
			 	    // Eikonal Equation Update
			 		
			 		float updateDist ; 
			 		if ( delta >= 0.0f ) updateDist = ( minX + minY + (float)Math.sqrt(delta) )/2.0f ;
			 		else updateDist = Math.min(minX, minY) + metricMap[ newY ][ newX ] ; 
			 		distanceMap[ newY ][ newX ] = updateDist ; 
			 		
			 		Q.remove( new Point ( newX , newY ) ) ;
	    			Q.add( new Point ( newX , newY ) ) ;
		    	}
		    }
	 	}
	}
	
	public void calcGradientMap()
	{
		gradientMapX = new float [height][width] ; 
		gradientMapY = new float [height][width] ; 
		
		float gradientAuxX[][] = new float [height][width] ; 
		float gradientAuxY[][] = new float [height][width] ; 
		
		for ( int y = 0 ; y < height ; y++ )
		{
			for ( int x = 0 ; x < width ; x++ )
			{
				if ( x == 0 ) gradientMapX[y][x] = distanceMap[y][x+1] - distanceMap[y][x] ;
				else if ( x == width - 1 ) gradientMapX[y][x] = distanceMap[y][x] - distanceMap[y][x-1] ;
				else gradientMapX[y][x] = ( distanceMap[y][x+1] - distanceMap[y][x-1] )/2.0f;
			}
		}
		
		for ( int x = 0 ; x < width ; x++ )
		{
			for ( int y = 0 ; y < height ; y++ )
			{
				if ( y == 0 ) gradientMapY[y][x] = distanceMap[y+1][x] - distanceMap[y][x] ;
				else if ( y ==  height - 1 ) gradientMapY[y][x] = distanceMap[y][x] - distanceMap[y-1][x] ;
				else gradientMapY[y][x] = ( distanceMap[y+1][x] - distanceMap[y-1][x] )/2.0f;
			}
		}
		
		// Smooth gradient matrices by moving average
		for ( int y = 1 ; y < height - 1 ; y++ )
			for ( int x = 1 ; x < width - 1 ; x++ )
			{
				gradientAuxX[y][x] = (  gradientMapX[y][x] + 
										gradientMapX[y-1][x] +
										gradientMapX[y+1][x] + 
										gradientMapX[y][x-1] +
										gradientMapX[y][x+1] ) / 4.0f ;
				
				gradientAuxY[y][x] = (  gradientMapY[y][x] + 
										gradientMapY[y-1][x] +
										gradientMapY[y+1][x] + 
										gradientMapY[y][x-1] +
										gradientMapY[y][x+1] ) / 4.0f ;
			}
		
		for ( int y = 1 ; y < height - 1 ; y++ )
			for ( int x = 1 ; x < width - 1 ; x++ )
			{
				gradientMapX[y][x] = gradientAuxX[y][x] ;
				gradientMapY[y][x] = gradientAuxY[y][x] ;
			}
	}
	
	private int getColor ( float[][] map,  int x, int y )
	{
		Color c ; 
		if ( Math.abs( x - start.getX() ) < 10 && 
			 Math.abs( y - start.getY() ) < 10 )
		{ 
			//Check if starting point
			c = Color.BLUE ;
			return c.getRGB() ; 
		}
		else if ( Math.abs( x - target.getX() ) < 10 && 
				  Math.abs( y - target.getY() ) < 10 ) 
		{  
			// Check if target point 
			c = Color.RED ; 
			return c.getRGB() ; 
		}
		else if ( map[ y ][ x ] < 0.0f )
		{
			// Geodesic Line
			c = Color.GREEN ; 
			return c.getRGB() ;
		}
		
		// Otherwise gray scale to describe the metricMap
		int rgbNum = 255 - (int) ( map[y][x]* 255.0 );
  	    c = new Color ( rgbNum, rgbNum, rgbNum ) ;
  	    return c.getRGB() ;
	}
	
	public float biLinInter ( int y1 , int x1 , int y2 , int x2 , 
							              float y , float x , float[][] map )
	{ 
		
		float aux1 = ( (float)( x - x1 )*map[y1][x2] + (float)( x2-x )*map[y1][x1] ) / (float)( x2-x1 ) ;
		float aux2 = ( (float)( x - x1 )*map[y2][x2] + (float)( x2-x )*map[y2][x1] ) / (float)( x2-x1 ) ;
		float ans  = ( (float)( y - y1 )*aux2 + (float)( y2-y )*aux1 ) / (float)( y2-y1 ) ;
		return ans ; 
	}
	
	public void plotGeodesic ( )
	{
		calcDistanceMap() ;
		
		int targetX = target.getX() ; 
		int targetY = target.getY() ; 
		
		float geodesicMap [][] = metricMap ; 
 
		float currX = (float)targetX ; 
		float currY = (float)targetY ; 
		
		int pixelX = targetX ; 
		int pixelY = targetY ; 
		
		// Gradient Descent
		
		calcGradientMap() ; // Gradient Calculation
		float tau = 0.2f ; 

		while (  Math.abs( currX - start.getX() ) > 1.0f || Math.abs( currY - start.getY() ) > 1.0f )
		{
			// Bilinear Interpolation
			int y2 = (int)Math.ceil(currY) ; int y1 = y2 - 1 ; 
			int x2 = (int)Math.ceil(currX) ; int x1 = x2 - 1 ; 
			
			float gradX = biLinInter(y1,x1,y2,x2,currY,currX,gradientMapX ) ;
			float gradY = biLinInter(y1,x1,y2,x2,currY,currX,gradientMapY ) ;

			float modulus = ( float )Math.sqrt ( gradX*gradX + gradY*gradY ) ; 
			gradX /= modulus ; gradY /= modulus ; 
			
			currX -= tau * gradX ; 
			currY -= tau * gradY ; 
			
			pixelX = ( int )( currX ); 
			pixelY = ( int )( currY ); 
			
			for ( int i = -1 ; i < 1 ; i++ )
				for ( int j = -1 ; j < 1 ; j++)
					geodesicMap[ pixelY + i ][ pixelX + j ] = -1000.0f ; 
		}
		
		plotMap ( geodesicMap , "Geodesic Plot" );
	}
		
	public void plotMap ( float[][] map, String imagename )
	{
		// Create image
		BufferedImage image0;
	    image0 = new BufferedImage(width, height,BufferedImage.TYPE_INT_ARGB ) ;
	    
	    // Normalize map
	 	 float maximum = 0.0f ; 
	 	 for ( int i = 0 ; i < height ; i++ )
	 	 	for ( int j = 0 ; j < width ; j++ )
	 	 		maximum = Math.max(maximum, map[i][j] ) ;
	 	 	
	 	 for ( int i = 0 ; i < height ; i++ )
	 	 	for ( int j = 0 ; j < width ; j++ )
	 	 		map[i][j] /= maximum ;
	       
	     //Writing Image
	     for ( int y = 0 ; y < height ; y++ )
	        for ( int x = 0 ; x < width ; x++ )
	        	image0.setRGB(x, y, getColor(map,x,y) );
      
	     // Saving picture using png format
	     File f = new File(imagename);

	     try{ ImageIO.write(image0, "png", f); } catch (IOException ex) 
	     { ex.printStackTrace(); }
	}
}
