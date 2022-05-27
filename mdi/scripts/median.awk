BEGIN {
# runs before input is read
FS=","
MEDIAN_LENGTH=10
}


{
	time[NR] = $2
}

function round(n) {
	n = n + 0.5
	n = int(n)
	return n
}

function floor(n) {
	return int(n)
}

function ceil(n) {
	n = n + 1
	return int(n)
}

function odd(n) {
	return n % 2 != 0
}

function even(n) {
	return n % 2 == 0
}

function mean(numbers) {
	sum = 0
	for (n in numbers) {
		sum += n
	}

	return sum / length(numbers)
}

function min(numbers) {
	m = numbers[0]
	for (n in numbers) {
		if (n < m) {
			m = n
		}
	}
	return m
}

function max(numbers) {
	m = numbers[0]
	for (n in numbers) {
		if (n > m) {
			m = n
		}
	}
	return m
}


# pre: assume 1 indexed array
function median(numbers) {
	l = length(numbers)
	print numbers | "sort -n" numbers
	if (even(l)) {
		return (numbers[l/2] + numbers[l/2 + 1]) / 2
	}
	else {
		return (numbers[l/2 + 1])
	}
}


END {
	# runs after input has been read, or exit
	i = 1
	for(; i < length(numbers); i *= MEDIAN_LENGTH) {
		for(j = 1; j < n; ++j) {

		}
	}

	for (t in time) {

	}
}

