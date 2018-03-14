
#ifndef BSP_ENCODERABZ_H_
#define BSP_ENCODERABZ_H_

enum
{
	ENCODER_1 = 0,
	ENCODER_2,
	ENCODER_MAX
};

class EncoderABZ {
public:
	EncoderABZ(uint32_t id);
	virtual ~EncoderABZ();

	int32_t GetDeltaStep();

protected:
	uint32_t id;

	int32_t GetAbsoluteStep();
};

#endif /* BSP_ENCODERABZ_H_ */
