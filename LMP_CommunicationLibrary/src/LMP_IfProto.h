#ifndef __LMP_IF_PROTO_H__
#define __LMP_IF_PROTO_H__


#include <stdint.h>

struct CanId {
	enum {
		ID_INVALID = 0x40000000,
		ID_EXT     = 0x80000000,
	};

	uint32_t id;

	void set_invalid() {
		id = ID_INVALID;
	}
	void set_std(uint16_t id) {
		this->id = id;
	}
	void set_ext(uint32_t id) {
		this->id = id | ID_EXT;
	}

	bool is_valid() const { return !(id & ID_INVALID); }
	bool is_ext() const { return id & ID_EXT; }
	uint32_t get_id() const { return id & 0x1FFFFFFF; }

	// Raw serialization
	uint32_t operator=(uint32_t id) {
		this->id = id;
		return id;
	}
	operator uint32_t() const {
		return id;
	}
};


#pragma pack(push, 1)

struct IfProtoConfig
{
	enum { type = 0x30 };

	enum EIFace
	{
		IFACE_UART_EXT_MAIN = 0,
		IFACE_UART_EXT_AUX  = 1,
		IFACE_UART_INT_SNS  = 5,
		IFACE_UART_INT_2    = 6,
		IFACE_UART_INT_3    = 7,
		IFACE_CAN1          = 10,
		IFACE_CAN2          = 11,
	};

	enum
	{
		length_unsupported   = 4,

		CMD_CONFIG           = 1,
		CMD_MESSAGE          = 10, // используется index
		CMD_FORWARD          = 11,
	};

	// Обязательная часть
	uint8_t iface;
	uint8_t cmd;
	uint8_t index;
	uint8_t reserved;
	// Переменная часть
	// Размер соответствует размеру структуры, если = 0 - то это запрос
	// Если на запрос возвращается длина 0 - данный тип не поддерживается
	uint8_t payload[128];

	//Q_GADGET
	//Q_ENUM(EIFace)
};

#pragma pack(2)

struct IfCanConfig
{
	enum
	{
		BAUD_OFF   = 0,
		BAUD_1000k = 1000,
		BAUD_500k  = 500,
		BAUD_250k  = 250,
		BAUD_125k  = 125,
		BAUD_100k  = 100,
		BAUD_50k   = 50,
	};
	enum EProto
	{
		PROTO_DISABLED                  = 0,
		PROTO_RX_PASSTHROUGHT           = 1,
		PROTO_RX_CARCAN_FORD            = 2,
		PROTO_RX_CARCAN_KIA_RIO_15      = 3,
		PROTO_RX_CARCAN_RENAULT_LOGAN2  = 4,
	};

	uint16_t baudrate_k; // 0 - отключен
	uint16_t proto;
	CanId io_id;         // !is_valid() (установлен 30 бит) - отключен

	//Q_GADGET
	//Q_ENUM(EProto)
	//Q_PROPERTY(unsigned baudrate_k MEMBER baudrate_k)
	//Q_PROPERTY(unsigned proto MEMBER proto)
	//Q_PROPERTY(unsigned io_id MEMBER io_id)
};
struct IfCanMessage
{
	enum EIndex
	{
		INDEX_STATUS_CNT = 0x01, // Статус и счетчик 1000
		INDEX_Wb         = 0x11, // Угловая скорость 1000
		INDEX_Ab         = 0x12, // Акселерометр 1000
		INDEX_Mb         = 0x13, // Магнитометр 1000
		INDEX_BARO       = 0x14, // Барометр 1000
		INDEX_TEMP       = 0x15, // Температура 1000

		INDEX_EULER      = 0x21, // Ориентация углы Эйлера 100
		INDEX_QUAT       = 0x22, // Ориентация кватернион 100
		INDEX_INCLINO    = 0x23, // Инклинометр 100

		INDEX_ALG_STATUS = 0x30, // Статус алгоритма
		INDEX_LATLON     = 0x31, // Координаты широта долгота 100
		INDEX_ALT        = 0x32, // Координаты высота 100
		INDEX_NED_XY     = 0x33, // Координаты в NED X,Y 100
		INDEX_Vn         = 0x34, // Скорость X,Y,Z 10
	};

	uint16_t prescaler; // 0 - сообщение отключено
	CanId id;

	//Q_GADGET
	//Q_ENUM(EIndex)
	//Q_PROPERTY(unsigned prescaler MEMBER prescaler)
	//Q_PROPERTY(unsigned id MEMBER id)
};

struct IfUartConfig
{
	enum EBaudRate
	{
		BAUDRATE_921600  = 0,
		BAUDRATE_460800  = 1,
		BAUDRATE_230400  = 2,
		BAUDRATE_115200  = 3,
		BAUDRATE_1000000 = 4,
		BAUDRATE_2000000 = 5,
		BAUDRATE_3000000 = 6,
		BAUDRATE_4000000 = 7,
		BAUDRATE_500000  = 8,
		BAUDRATE_57600   = 9,
		BAUDRATE_38400   = 10,
		BAUDRATE_19200   = 11,
		BAUDRATE_9600    = 12,
	};

	enum EProto
	{
		PROTO_DISABLED            = 0,
		PROTO_GV                  = 1,
		PROTO_GV_DATA             = 2,
		PROTO_PASSTHROUGH_TO_MAIN = 3,
		PROTO_NMEA_FROM_GNSS      = 4,

		PROTO_GNSS_NV08C          = 10, 
		PROTO_GNSS_GEOS3M         = 11,
		PROTO_GNSS_NMEA           = 12,
		PROTO_GNSS_COMNAV         = 13,
		PROTO_GNSS_NOVATEL        = 14,
		PROTO_GNSS_MNP            = 15,
		PROTO_GNSS_SEPTENTRIO     = 16,
		PROTO_GNSS_UBLOX          = 20,

		PROTO_ETC                 = 100,
	};

	enum EMode
	{
		// Параметры порта
		MODE_UART                = 0 << 0,
		MODE_RS232               = 1 << 0,
		MODE_RS485               = 2 << 0,
		MODE_TYPE_MASK           = 3 << 0,
	};

	// Параметры конкретных протоколов
	enum EProtoParam
	{
		PARAM_USE_GNSS_DATA       = 1 << 0,
		PARAM_USE_GNSS_RELPOS     = 1 << 1,
		PARAM_TO_ESKF_GNSS_DATA   = 1 << 2,
		PARAM_TO_ESKF_GNSS_RELPOS = 1 << 3,

		PARAM_NMEA_PRESCALER_MASK = 7 << 0,
		PARAM_NMEA_PRESCALER_1    = 0 << 0,
		PARAM_NMEA_PRESCALER_5    = 1 << 0,
		PARAM_NMEA_PRESCALER_10   = 2 << 0,
		PARAM_NMEA_PRESCALER_50   = 3 << 0,
		PARAM_NMEA_PRESCALER_100  = 4 << 0,
		PARAM_NMEA_PRESCALER_500  = 5 << 0,
		PARAM_NMEA_PRESCALER_1000 = 6 << 0,
	};

	uint16_t baudrate;
	uint8_t proto;
	uint8_t proto_param;
	uint16_t mode;

	int nmea_prescaler() const
	{
		int v = proto_param & PARAM_NMEA_PRESCALER_MASK;
		int p = v & 1 ? 5 : 1;
		switch (v >> 1)
		{
		case 3: p *= 10;
		case 2: p *= 10;
		case 1: p *= 10;
		}
		return p;
	}

	//Q_GADGET
	//Q_ENUM(EBaudRate)
	//Q_ENUM(EProto)
	//Q_ENUM(EMode)
	//Q_ENUM(EProtoParam)
	//Q_PROPERTY(unsigned baudrate MEMBER baudrate)
	//Q_PROPERTY(unsigned proto MEMBER proto)
	//Q_PROPERTY(unsigned proto_param MEMBER proto_param)
	//Q_PROPERTY(unsigned mode MEMBER mode)
};
struct IfUartForward
{
	uint16_t buf_size; // 0 - отключено
	uint8_t iface;

	//Q_GADGET
	//Q_PROPERTY(unsigned buf_size MEMBER buf_size)
	//Q_PROPERTY(unsigned iface MEMBER iface)
};

namespace CanMsg {
	// Выходные сообщения
	struct StatusCnt
	{
		enum { index = IfCanMessage::INDEX_STATUS_CNT };

		uint32_t sample_cnt;
		uint16_t status;
	};
	struct Wb
	{
		enum { index = IfCanMessage::INDEX_Wb };

		uint16_t w[3];
	};
	struct Ab
	{
		enum { index = IfCanMessage::INDEX_Ab };

		uint16_t a[3];
	};
	struct Mb
	{
		enum { index = IfCanMessage::INDEX_Mb };

		uint16_t m[3];
	};
	struct Baro
	{
		enum { index = IfCanMessage::INDEX_BARO };

		uint32_t baro;
	};
	struct Euler
	{
		enum { index = IfCanMessage::INDEX_EULER };

		int16_t yaw;
		int16_t pitch;
		int16_t roll;
	};
	struct Inclino
	{
		enum { index = IfCanMessage::INDEX_INCLINO };

		int16_t alfa;
		int16_t beta;
	};
	struct AlgStatus
	{
		enum { index = IfCanMessage::INDEX_ALG_STATUS };

		int8_t stage;
		int8_t update;
		int8_t fails;
	};

	struct Io {
		enum {
			INDEX_CMD                       = 0x10,

			// Списать смещение гироскопов.
			// Параметр 4 байта data[0..3] uint32 - количество сэмплов для усреднения
			SUBINDEX_CMD_ADJUST_GYRO_OFFSET = 0x01,
			// Сброс алгоритма
			// Параметра нет
			SUBINDEX_CMD_RESET_ALGORITHM    = 0x10,
		};

		uint8_t index;
		uint8_t subindex;
		uint8_t data[6]; // Размер может быть меньше, зависит от index

		uint32_t get_u32() const {
			return data[0] | (data[1] << 8) | (data[2] << 16) | (data[3] << 24);
		}
	};
} // namespace CanMsg


#pragma pack(pop)



#endif // proto_gyrovert_ifproto_h__
