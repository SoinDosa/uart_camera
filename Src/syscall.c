int _open_r(void *reent, const char *file, int flags, int mode){
	return 0;
}
int _close_r(void *reent, int fd){
	return 0;
}
int _lseek_r(void *reent, int fd, int pos, int whence){
	return 0;
}
long _read_r(void *reent, int fd, void *buf, unsigned int cnt){
	return 0;
}
long _write_r(void *reent, int fd, const void *buf,  unsigned int cnt){
	return 0;
}

int _fstat (int fd, void* st){
 return 0;
}

int _getpid_r (void *r)
{
  return 0;
}

int _isatty_r( void *ptr, int fd) {
	return 0;
}

int _kill_r(void *ptr, int pid, int sig  )
{
	return 0;
}
