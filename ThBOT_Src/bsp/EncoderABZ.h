
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
	double  GetAbsoluteMM();

protected:
	uint32_t id;

//	int32_t GetAbsoluteStep();
private:
	int32_t Absolute_value;
	double  MM_per_step;
};

#endif /* BSP_ENCODERABZ_H_ */
