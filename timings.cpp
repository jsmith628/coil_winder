
#include <SoftwareSerial.h>
#include "timings.h"

int euclid(int n, int m) {

  int r = n % m;
  if(r==0) {
    return m;
  } else {
    return euclid(m,r);
  }

}

int gcd(int n, int m) {

  if(n==0 || m==0) return 0;

  if(n >= m){
    return euclid(n,m);
  } else {
    return euclid(m,n);
  }

}

int lcm(int n, int m) {
  return n * m / gcd(n,m);
}
