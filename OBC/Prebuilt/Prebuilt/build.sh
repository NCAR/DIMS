exceptionFiles=("main.c" "ESTTC.c" "MCU_Init.c" "EEPROM_emul.c" "AppTasks.c")
find $2 -name "*.c" | while read i
do
{
	f="$(basename -- $i)"
	if [[ " ${exceptionFiles[@]} " =~ " $f " ]]
	then
	{
		ofile="$(echo $f | cut -f1 -d.)"
		echo REMOVING $2/Prebuilt/$1/$ofile.o
		rm -f $2/Prebuilt/$1/$ofile.o
		echo KEEPING FILE $i
	}
	else
	{
		echo DELETING FILE $i
		rm -f $i
	}
	fi
}
done
rm -f $2/Prebuilt/$1/startup_stm32f427xx.o
sleep 5s
echo BUILD TREE PREPARED.
