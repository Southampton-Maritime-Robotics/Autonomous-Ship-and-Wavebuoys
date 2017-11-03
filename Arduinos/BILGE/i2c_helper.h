//***************************************
// i2c transmission helper functions

template <class T>
int WireWriteAnything(const T & value)
{  
  union val_split
  {
    byte b[sizeof(value)];
    T fval;
  } x;
  x.fval = value;
  int i;
  for (i = 0; i < sizeof(value); i++)
    Wire.write(x.b, sizeof(value));
  //Serial.println(x.b[1]);
  return i; // number of bytes sent
}

template <class T>
int WireReadAnything(T & value)
{
  union val_split
  {
    byte b[sizeof(value)];
    T fval;
  } x;
  int i;
  for (i = 0; i < sizeof(value); i++)
      x.b[i] = Wire.read();
  value = x.fval;
  return i;
}
