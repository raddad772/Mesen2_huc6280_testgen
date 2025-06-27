#pragma once
#include "pch.h"
#include "PCE/PceTypes.h"
#include "PCE/IPceMapper.h"
#include "Shared/Emulator.h"
#include "Shared/CheatManager.h"
#include "Utilities/HexUtilities.h"
#include "Utilities/ISerializable.h"
#include "Shared/MemoryOperationType.h"

class PceConsole;
class PceVpc;
class PceVce;
class PcePsg;
class PceControlManager;
class PceCdRom;
class PceTimer;

class PceMemoryManager final : public ISerializable
{
public:
    //private:
	Emulator* _emu = nullptr;
	CheatManager* _cheatManager = nullptr;
	PceConsole* _console = nullptr;
	PceVpc* _vpc = nullptr;
	PceVce* _vce = nullptr;
	PcePsg* _psg = nullptr;
	PceControlManager* _controlManager = nullptr;
	PceCdRom* _cdrom = nullptr;
	PceTimer* _timer = nullptr;
	IPceMapper* _mapper = nullptr;

	typedef void(PceMemoryManager::*Func)();
	Func _exec = nullptr;
	Func _fastExec = nullptr;

	PceMemoryManagerState _state = {};
	uint8_t* _prgRom = nullptr;
	uint32_t _prgRomSize = 0;
	
	uint8_t* _readBanks[0x100] = {};
	uint8_t* _writeBanks[0x100] = {};
	MemoryType _bankMemType[0x100] = {};

	uint8_t* _workRam = nullptr;
	uint32_t _workRamSize = 0;

	uint8_t* _cardRam = nullptr;
	uint32_t _cardRamSize = 0;
	uint32_t _cardRamStartBank = 0;
	uint32_t _cardRamEndBank = 0;

	uint8_t* _unmappedBank = nullptr;
	uint8_t* _saveRam = nullptr;
	uint8_t* _cdromRam = nullptr;
	
	bool _cdromUnitEnabled = false;

public:
	PceMemoryManager(Emulator* emu, PceConsole* console, PceVpc* vpc, PceVce* vce, PceControlManager* controlManager, PcePsg* psg, PceTimer* timer, IPceMapper* mapper, PceCdRom* cdrom, vector<uint8_t>& romData, uint32_t cardRamSize, bool cdromUnitEnabled);
	~PceMemoryManager();

	PceMemoryManagerState& GetState() { return _state; }

	void SetSpeed(bool slow);
	void UpdateMappings(uint32_t bankOffsets[8]);
	void UpdateCdRomBanks();

	void UpdateExecCallback();
	
	template<bool hasCdRom, bool isSuperGrafx> void ExecTemplate();

	void ExecSlow();

	__forceinline void Exec() { (this->*_exec)(); }
	__forceinline void ExecFastCycle() { (this->*_fastExec)(); }

	uint8_t Read(uint16_t addr, MemoryOperationType type = MemoryOperationType::Read);
	void Write(uint16_t addr, uint8_t value, MemoryOperationType type);

	uint8_t ReadRegister(uint16_t addr);
	void WriteRegister(uint16_t addr, uint8_t value);
	void WriteVdc(uint16_t addr, uint8_t value);

	uint8_t DebugRead(uint16_t addr);
	void DebugWrite(uint16_t addr, uint8_t value);

	void SetMprValue(uint8_t regSelect, uint8_t value);
	uint8_t GetMprValue(uint8_t regSelect);

	AddressInfo GetAbsoluteAddress(uint32_t relAddr);
	AddressInfo GetRelativeAddress(AddressInfo absAddr, uint16_t pc);

	void SetIrqSource(PceIrqSource source) { _state.ActiveIrqs |= (int)source; }
	__forceinline uint8_t GetPendingIrqs() { return (_state.ActiveIrqs & ~_state.DisabledIrqs); }
	__forceinline bool HasIrqSource(PceIrqSource source) { return (_state.ActiveIrqs & ~_state.DisabledIrqs & (int)source) != 0; }
	void ClearIrqSource(PceIrqSource source) { _state.ActiveIrqs &= ~(int)source; }

	void Serialize(Serializer& s);
};

