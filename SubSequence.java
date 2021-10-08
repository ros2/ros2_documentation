import java.io.*;
import java.util.*;

public class Main {

    public static void main(String[] args) throws Exception {
        Scanner scn =new Scanner (System.in);
        String s=scn.next();
        ArrayList<String> al = gss(s);
        System.out.print(al);
        
        

    }

    public static ArrayList<String> gss(String str) {
        if(str.length()==0){
            ArrayList<String> bs=new ArrayList<>();
            bs.add("");
            return bs;
        }
        char ch=str.charAt(0);
        String st=str.substring(1);
        ArrayList<String> rss=gss(st);
        ArrayList<String> mr=new ArrayList<>();
        
        for(String val: rss){
          mr.add(""+val);
        }
        for(String val: rss){
            mr.add(ch+val);
        }
        return mr;
        
        
         
    }

}
