class PVector{
	
	
public:
	float x;
	float y;
	float z;

	//PVector(){}
	PVector(float a=0,float b=0,float c=0){
		x=a;
		y=b;
		z=c;
	}
	
	void set(float a=0,float b=0,float c=0){
		x=a;
		y=b;
		z=c;
	}
	
	void set(PVector a){
		x=a.x;
		y=a.y;
		z=a.z;
	}
	
	void add(float a=0,float b=0,float c=0){
		x+=a;
		y+=b;
		z+=c;
	}
	void add(PVector a){
		x+=a.x;
		y+=a.y;
		z+=a.z;
	}
	
	void sub(PVector a){
		x-=a.x;
		y-=a.y;
		z-=a.z;
	}
	
	float magSq(){
		float ret = 0;
		ret += x*x;
		ret += y*y;
		ret += z*z;
		return ret;
	}
	
};



