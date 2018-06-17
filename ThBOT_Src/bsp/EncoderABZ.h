
#ifndef BSP_ENCODERABZ_H_
#define BSP_ENCODERABZ_H_

typedef enum
{
	ENCODER_LEFT = 0,
	ENCODER_RIGHT,
	ENCODER_MAX
} Encodeur_t;

class EncoderABZ {
public:
	EncoderABZ(Encodeur_t id);
	virtual ~EncoderABZ();

	int32_t GetDeltaStep();
	double  GetDeltaMM();

	int32_t GetAbsoluteStep();
	int32_t GetAbsoluteStepFromDelta();
	double  GetAbsoluteMM();

protected:
	uint32_t id;

//	int32_t GetAbsoluteStep();
private:
	int32_t LastAbsoluteValue;
	int32_t AbsoluteStepFromDelta;
	double  MM_per_step;
};

#endif /* BSP_ENCODERABZ_H_ */
