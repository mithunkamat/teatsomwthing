


#define GCC_VERSION_MAJOR __GNUC__ * 10000
#define GCC_VERSION_MINOR __GNUC_MINOR__ * 100
#define GCC_VERSION_PATCHLEVEL __GNUC_PATCHLEVEL__

#define GCC_VERSION (__GNUC__ * 10000 \
         + __GNUC_MINOR__ * 100 \
         + __GNUC_PATCHLEVEL__)

int main(void)
{

	serialInit();
	printf("\n\nApplication:\n");

}

