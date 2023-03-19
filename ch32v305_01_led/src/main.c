#include "main.h"




int _main(void)
{
  hwInit();
  apInit();
  apMain();

  return 0;
}

int main(void)
{
  _main();
  return 0;
}