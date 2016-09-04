package geodesics;

import java.lang.Math ;

public class Test 
{
	 public static void main(String[] args)
	 {
	        // Set up a metric map
	        float Map[][] = new float [1200][1200] ; 
	        
	        for ( int i = 0 ; i < 1200 ; i++ )
	        {
	        	for ( int j = 0 ; j < 1200 ; j++ )
	        	{
	        		float dx = ( float ) ( i - 600.0f ) / 1200.0f ;
	        		float dy = ( float ) ( j - 600.0f ) / 1200.0f ; 
	        		
	        		Map[i][j] = ( float ) ( 2.0f + 3.0 * Math.exp( ( -dx*dx - dy*dy ) / 0.05f ) ) ;
	        	}
	        }
	        
	        Point start = new Point ( 100 , 200 ) ; 
	        Point target = new Point ( 950 , 900 ) ;
	        Geodesic g = new Geodesic ( Map , start , target );
	        g.plotGeodesic();
	 }
}
