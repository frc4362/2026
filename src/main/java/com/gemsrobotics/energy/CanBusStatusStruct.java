package com.gemsrobotics.energy;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructGenerator;

import java.nio.ByteBuffer;

public final class CanBusStatusStruct implements Struct<CANBus.CANBusStatus> {
	public static final Struct<StatusCode> STATUS_CODE_STRUCT = StructGenerator.genEnum(StatusCode.class);
	public static final CanBusStatusStruct struct = new CanBusStatusStruct();

	@Override
	public Class<CANBus.CANBusStatus> getTypeClass() {
		return CANBus.CANBusStatus.class;
	}

	@Override
	public String getTypeName() {
		return "CanBusStatus";
	}

	@Override
	public int getSize() {
		return Integer.BYTES * 4 + Struct.kSizeFloat + STATUS_CODE_STRUCT.getSize();
	}

	@Override
	public String getSchema() {
		return STATUS_CODE_STRUCT.getTypeName() + " statusCode; float busUtilization; int32 busOffCount; int32 txFullCount; int32 receiveErrorCount; int32 transmitErrorCount";
	}

	@Override
	public Struct<?>[] getNested() {
		return new Struct<?>[]{ STATUS_CODE_STRUCT };
	}

	@Override
	public CANBus.CANBusStatus unpack(final ByteBuffer bb) {
		final CANBus.CANBusStatus ret = new CANBus.CANBusStatus();
		ret.Status = STATUS_CODE_STRUCT.unpack(bb);
		ret.BusUtilization = bb.getFloat();
		ret.BusOffCount = bb.getInt();
		ret.TxFullCount = bb.getInt();
		ret.REC = bb.getInt();
		ret.TEC = bb.getInt();
		return ret;
	}

	@Override
	public void pack(final ByteBuffer bb, final CANBus.CANBusStatus value) {
		STATUS_CODE_STRUCT.pack(bb, value.Status);
		bb.putFloat(value.BusUtilization);
		bb.putInt(value.BusOffCount);
		bb.putInt(value.TxFullCount);
		bb.putInt(value.REC);
		bb.putInt(value.TEC);
	}
}
