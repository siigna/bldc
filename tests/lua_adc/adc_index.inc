static int adc_index(int channel, bool *aliased) {
	int ind;

	*aliased = false;
	switch (channel) {
	case 0: return ADC_IND_EXT;
	case 1: ind = ADC_IND_EXT2; break;
	case 2: ind = ADC_IND_EXT3; break;
	case 3: return ADC_IND_TEMP_MOTOR;
	case 4: ind = ADC_IND_EXT4; break;
	case 5: ind = ADC_IND_EXT5; break;
	case 6: ind = ADC_IND_EXT6; break;
	case 7: ind = ADC_IND_EXT7; break;
	case 8: ind = ADC_IND_EXT8; break;
	default: return -1;
	}

	if (ind == ADC_IND_EXT) {
		*aliased = true;
	}
	return ind;
}
