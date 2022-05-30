void main(void)
{
	Initialization();

	do
	{
		Import_Data();

		Controller();

		Export_Data();

		Time_Management();
		//Static_Characteristic();
		
	} while (Check_Stop_Condition());

	File_Writing();

	DAQ_Terminatation();

	system("pause");
}