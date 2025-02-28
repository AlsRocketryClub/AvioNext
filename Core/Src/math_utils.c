void multiplyQuat(double r[4], double s[4], double *result) {
	double temp[4];
	temp[0] = r[0] * s[0] - r[1] * s[1] - r[2] * s[2] - r[3] * s[3];
	temp[1] = r[0] * s[1] + r[1] * s[0] - r[2] * s[3] + r[3] * s[2];
	temp[2] = r[0] * s[2] + r[1] * s[3] + r[2] * s[0] - r[3] * s[1];
	temp[3] = r[0] * s[3] - r[1] * s[2] + r[2] * s[1] + r[3] * s[0];

	for (int i = 0; i < 4; i++) {
		result[i] = temp[i];
	}
}

double dotProduct(double a[4], double b[4]) {
	return (a[1] * b[1] + a[2] * b[2] + a[3] * b[3]);
}

double magnitude(double vector[4]) {
	return sqrt(
			vector[0] * vector[0] + vector[1] * vector[1]
					+ vector[2] * vector[2] + vector[3] * vector[3]);
}