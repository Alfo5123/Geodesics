package geodesics;

public class Point 
{
	int x ; 
	int y ; 

	public Point (){} ; 
	
	public Point ( int x , int y ) 
	{
		this.x = x ; 
		this.y = y ;
	}
	
	int getX ( )
	{
		return this.x ; 
	}
	
	int getY ( )
	{
		return this.y ;
	}
	
}
