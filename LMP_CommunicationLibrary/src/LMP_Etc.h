#ifndef LMP_ETC_H
#define LMP_ETC_H

#include <stdint.h>

#pragma pack(push, 2)

struct Measurement {
	enum { type = 0x44 };

	enum ETimestampType {
		TIMESTAMP_NOW = 0,
		TIMESTAMP_OLDEST_IN_QUEUE = 1,
		TIMESTAMP_SAMPLE_CNT = 2,
		TIMESTAMP_CNT_SINCE_SEC = 3,
	};

	enum EStatus {
		STATUS_OK = 0,
		STATUS_TIMESTAMP_UNMATCH = 1,
		STATUS_ERROR = 2,
	};

	struct Yaw {
		enum { type = 1 };
		float yaw;
		float sig;
	};

	struct YawPitchRoll {
		enum { type = 2 };
		float yaw;
		float pitch;
		float roll;
		float sig_yaw;
		float sig_pitch;
		float sig_roll;
	};

	struct LLA {
		enum { type = 3 };
		double lla[3];
		float sig[3];
	};

	struct Velocity {
		enum { type = 4 };
		float vel[3];
		float sig[3];
	};

	struct LLAVelocity {
		enum { type = 5 };
		double lla[3];
		float vel[3];
		float sig_lla[3];
		float sig_vel[3];
	};

	struct VelocityBody {
		enum { type = 6 };
		float vel[3];
		float sig[3];
	};

	uint16_t param;
	uint16_t status;
	uint16_t timestamp;
	uint16_t reserved;
	uint8_t payload[96];

	Measurement() 
		: param(0)
		, status(0)
		, timestamp(0)
		, reserved(0)
	{}

	void set_type(int type) {
		param = (param & ~0x001F) | type;
	}

	int get_type() const {
		return param & 0x001F;
	}

	void set_timestamp_type(ETimestampType ts) {
		param = (param & ~0x0060) | (ts << 5);
	}

	ETimestampType get_timestamp_type() const {
		return static_cast<ETimestampType>((param & 0x0060) >> 5);
	}

	uint32_t length() const {
		const int payload_size[] = {
			0,
			sizeof(Yaw),
			sizeof(YawPitchRoll),
			sizeof(LLA),
			sizeof(Velocity),
			sizeof(LLAVelocity),
			sizeof(VelocityBody),
		};
		int t = get_type();
		if (t >= sizeof(payload_size) / sizeof(payload_size[0])) {
			t = 0;
		}
		return 8 + payload_size[t];//4 + payload_size[t]; default value
	}

	template<typename T>
	T &get_payload() {
		(void)T::type;
		return *reinterpret_cast<T *>(&payload[0]);
	}
	template<typename T>
	const T &get_payload() const {
		(void)T::type;
		return *reinterpret_cast<const T *>(&payload[0]);
	}
};


#pragma pack(pop)

#endif // proto_gyrovert_etc_h__
