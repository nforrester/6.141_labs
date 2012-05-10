
public class DummyTestClass {


	public static void main(String[] args){
		
		System.out.println("Hekko worlds raga".split("\\s")[1]);

		boolean[][]  values= new boolean[][]{{false,false,false,false,false,false,false,false,false,false},
				{false,false,true,true,true,true,true,false,false,false},
				{false,false,true,true,true,true,true,false,false,false},
				{false,false,true,true,true,true,false,false,false,false},
				{false,false,false,true,true,false,false,false,false,false},
				{false,false,false,true,true,false,false,false,false,false},
				{false,false,false,false,false,false,false,false,false,false},
				{false,false,false,false,false,false,false,true,true,true},
				{false,false,false,false,false,false,false,true,true,true},
				{false,false,false,false,false,false,false,true,true,true}};



		for(int i =0; i<10;i++){
			for(int j =0; j<10;j++){
				boolean one = false;
				boolean two = false;
				boolean three = false;
				if(i+1 < 10){
					if((values[i][j] && values[i+1][j])){
						one = true;
					}
				}
				if(j +1 <10){
					if((values[i][j] && values[i][j+1])){
						two = true;
					}
				}
				if(i+1 <10 && j+1 < 10){
					if((values[i][j] && values[i+1][j + 1])){
						three = true;
					}
				}
				if(one || two || three){
					values[i][j] = false;
				}
			}
		}


		for(boolean[] a : values){
			for(boolean b : a){
				System.out.print(b + ",");
			}
			System.out.println();
		}

	}

}
